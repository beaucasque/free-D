"""
lensaxis.py — Extraction d'un scalaire d'objectif a partir de deux poses Vive.

PRINCIPE

Un tracker monte sur une roulette de follow focus donne 6DoF. On veut un seul
nombre : la position de la bague. Quatre etapes.

1. SOUSTRACTION DU MOUVEMENT CAMERA
   La camera bouge pendant la prise. La rotation absolue du tracker objectif
   ne veut donc rien dire : un panoramique la fait tourner autant qu'un coup
   de point. On travaille sur

       q_rel = conj(q_camera) * q_objectif

   Tout mouvement commun aux deux trackers s'annule exactement. Travelling,
   panoramique, grue : le point ne bouge pas.

   La TRANSLATION n'a pas besoin d'etre soustraite pour l'angle — un
   quaternion ne sait rien de la position. Elle sert a autre chose, voir 4.

2. ALIGNEMENT TEMPOREL — indispensable des que la camera bouge vite
   Les deux trackers ne sont PAS echantillonnes au meme instant. Confronter
   le dernier q_camera connu a un q_objectif plus recent fait apparaitre une
   rotation relative parasite egale a la vitesse angulaire camera multipliee
   par le decalage. A 180 deg/s de panoramique et 4 ms d'ecart, c'est
   0.7 degre de faux mouvement de point — pile pendant le geste ou l'oeil
   est le plus sensible.

   CameraHistory garde un historique et rend q_camera INTERPOLE (slerp) a
   l'horodatage exact de l'echantillon objectif. C'est ce qui rend la
   soustraction reellement temps reel plutot qu'approximative.

3. AXE PAR BALAYAGE, PAS A LA REGLE
   On enregistre q_rel butee a butee, log-map de chaque increment, matrice
   N x 3, SVD. Le premier vecteur singulier est l'axe du pignon exprime dans
   le repere du tracker camera. Aucune mesure mecanique ; un montage de
   travers est encaisse.

   sigma[1]/sigma[0] est le verdict sur le montage : petit = rotation
   planaire, support rigide.

4. SWING-TWIST, ET LA POSITION COMME CHIEN DE GARDE
   On decompose l'increment en twist (autour de l'axe) et swing (flexion du
   support, jeu de roulement) et on ne garde que le twist.

   La position relative, elle, sert de controle d'integrite : la distance
   entre tracker camera et tracker objectif est une CONSTANTE MECANIQUE. Si
   elle derive, un support a glisse et l'axe calibre ne vaut plus rien.
   MountWatch le signale en temps reel — sinon la panne est silencieuse et
   ne se voit qu'au compositing.

MULTI-TOUR
   theta sort dans (-pi, pi]. Accumulator deroule. Si la course relevee au
   balayage reste sous 360 degres, aucune ambiguite : absolu au demarrage,
   pas de homing.

CONVENTION
   Quaternions (w, x, y, z), l'ordre de libsurvive (SurvivePose.Rot).
"""

import bisect
import json
import math

import numpy as np

# ---------------------------------------------------------------- quaternions


def q_norm(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_canon(q):
    """Force w >= 0. Un quaternion et son oppose codent la meme rotation ;
    sans ca le log-map saute de signe au milieu d'un balayage."""
    return q if q[0] >= 0.0 else (-q[0], -q[1], -q[2], -q[3])


def q_log(q):
    """Log-map : quaternion unitaire -> vecteur rotation (axe * angle)."""
    w, x, y, z = q_canon(q_norm(q))
    v = math.sqrt(x * x + y * y + z * z)
    if v < 1e-12:
        return (0.0, 0.0, 0.0)
    theta = 2.0 * math.atan2(v, w)
    k = theta / v
    return (x * k, y * k, z * k)


def q_slerp(a, b, u):
    """Interpolation spherique. u dans [0, 1]."""
    a = q_norm(a)
    b = q_norm(b)
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
    if dot < 0.0:                       # chemin court
        b = (-b[0], -b[1], -b[2], -b[3])
        dot = -dot
    if dot > 0.9995:                    # quasi colineaires : lineaire suffit
        return q_norm(tuple(a[i] + u * (b[i] - a[i]) for i in range(4)))
    th0 = math.acos(max(-1.0, min(1.0, dot)))
    s = math.sin(th0)
    s0 = math.sin((1.0 - u) * th0) / s
    s1 = math.sin(u * th0) / s
    return q_norm(tuple(s0 * a[i] + s1 * b[i] for i in range(4)))


def q_rotate(q, v):
    """Applique la rotation q au vecteur v."""
    t = q_mul(q_mul(q, (0.0, v[0], v[1], v[2])), q_conj(q))
    return (t[1], t[2], t[3])


def q_rate(a, b, dt):
    """Vitesse angulaire entre deux quaternions, en rad/s."""
    if dt <= 0.0:
        return 0.0
    d = q_mul(q_conj(q_norm(a)), q_norm(b))
    return math.sqrt(sum(c * c for c in q_log(d))) / dt


def relative(q_cam, q_lens):
    """q_rel = conj(q_camera) * q_objectif. Soustrait la rotation camera."""
    return q_norm(q_mul(q_conj(q_norm(q_cam)), q_norm(q_lens)))


def relative_position(q_cam, p_cam, p_lens):
    """Position de l'objectif dans le repere camera. Soustrait la translation
    ET la rotation camera. Constante mecanique : sert de controle d'integrite
    du montage, pas au calcul de l'angle."""
    d = (p_lens[0] - p_cam[0], p_lens[1] - p_cam[1], p_lens[2] - p_cam[2])
    return q_rotate(q_conj(q_norm(q_cam)), d)


def twist_angle(dq, axis):
    """Composante twist de la decomposition swing-twist, en radians.

    Retourne un angle dans (-pi, pi]. Le swing — desalignement, flexion du
    support — est ecarte et n'apparait pas dans le resultat.
    """
    w, x, y, z = q_canon(q_norm(dq))
    proj = x * axis[0] + y * axis[1] + z * axis[2]
    return 2.0 * math.atan2(proj, w)


# ------------------------------------------------------- alignement temporel


class CameraHistory:
    """Historique horodate des poses camera, interrogeable par slerp.

    Les trackers ne remontent pas en cadence commune. Sans interpolation, la
    soustraction du mouvement camera se fait avec un q_camera perime et le
    residu part directement dans le zoom et le focus.

    at(t) rend (q, p, etat) avec etat parmi :
        "exact"   echantillon encadre, interpole — le cas normal
        "extrap"  t posterieur au dernier echantillon camera : l'appelant
                  devrait differer d'un tick plutot que d'accepter
        "stale"   t anterieur a l'historique retenu
    """

    def __init__(self, span=1.0):
        self.span = span
        self.t = []
        self.q = []
        self.p = []

    def push(self, t, quat, pos):
        # Horodatages non monotones : on ignore, plutot que de casser bisect.
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t)
        self.q.append(q_norm(quat))
        self.p.append(tuple(pos))
        cut = t - self.span
        while len(self.t) > 2 and self.t[0] < cut:
            self.t.pop(0)
            self.q.pop(0)
            self.p.pop(0)

    def newest(self):
        return self.t[-1] if self.t else None

    def latest(self):
        return (self.q[-1], self.p[-1]) if self.t else None

    def at(self, t):
        if not self.t:
            return None
        if t >= self.t[-1]:
            return (self.q[-1], self.p[-1], "extrap")
        if t <= self.t[0]:
            return (self.q[0], self.p[0], "stale")
        i = bisect.bisect_left(self.t, t)
        t0, t1 = self.t[i - 1], self.t[i]
        u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        q = q_slerp(self.q[i - 1], self.q[i], u)
        p0, p1 = self.p[i - 1], self.p[i]
        p = tuple(p0[k] + u * (p1[k] - p0[k]) for k in range(3))
        return (q, p, "exact")

    def rate(self):
        """Vitesse angulaire camera instantanee, rad/s. Diagnostic : si le
        point ne bouge que quand ce chiffre monte, l'alignement temporel ou
        l'axe sont en cause."""
        if len(self.t) < 2:
            return 0.0
        return q_rate(self.q[-2], self.q[-1], self.t[-1] - self.t[-2])


class MountWatch:
    """Surveille la distance camera <-> objectif, constante mecanique.

    Une derive signifie qu'un support a glisse : l'axe calibre ne decrit plus
    la rotation reelle et le point devient faux sans que rien ne le dise.
    """

    def __init__(self, tolerance_mm=8.0):
        self.tolerance = tolerance_mm / 1000.0
        self.ref = None
        self.drift = 0.0

    def push(self, p_rel):
        d = math.sqrt(p_rel[0] ** 2 + p_rel[1] ** 2 + p_rel[2] ** 2)
        if self.ref is None:
            self.ref = d
            return False
        self.drift = d - self.ref
        return abs(self.drift) > self.tolerance


# ---------------------------------------------------------------- calibration


def fit_axis(quats_rel, ref=None):
    """Trouve l'axe de rotation a partir d'un balayage butee a butee.

    Retourne un dict : axis, ref, lo, hi, span_deg, planarity, rms_deg,
    samples.
    """
    if len(quats_rel) < 20:
        raise ValueError("balayage trop court : %d echantillons" % len(quats_rel))

    if ref is None:
        ref = quats_rel[0]
    ref = q_norm(ref)
    inv_ref = q_conj(ref)

    logs = np.array([q_log(q_mul(inv_ref, q_norm(q))) for q in quats_rel])

    # Les increments quasi nuls ne portent pas d'information de direction et
    # tirent la SVD vers le bruit : on les jette.
    mag = np.linalg.norm(logs, axis=1)
    keep = logs[mag > np.deg2rad(1.0)]
    if len(keep) < 10:
        raise ValueError("balayage insuffisant : la bague a-t-elle bouge ?")

    _u, s, vt = np.linalg.svd(keep, full_matrices=False)
    axis = vt[0] / np.linalg.norm(vt[0])

    # Orientation : theta doit croitre dans le sens du balayage.
    if float(np.sum(keep @ axis)) < 0.0:
        axis = -axis

    planarity = float(s[1] / s[0]) if s[0] > 0 else float("inf")
    residual = keep - np.outer(keep @ axis, axis)
    rms_deg = float(np.rad2deg(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))

    axis_t = (float(axis[0]), float(axis[1]), float(axis[2]))
    acc = Accumulator()
    thetas = [acc.push(twist_angle(q_mul(inv_ref, q_norm(q)), axis_t))
              for q in quats_rel]

    return {
        "axis": list(axis_t),
        "ref": list(ref),
        "lo": min(thetas),
        "hi": max(thetas),
        "span_deg": math.degrees(max(thetas) - min(thetas)),
        "planarity": planarity,
        "rms_deg": rms_deg,
        "samples": len(quats_rel),
    }


def verdict(cal):
    """Jugement lisible sur un resultat de fit_axis."""
    p, r = cal["planarity"], cal["rms_deg"]
    if p < 0.05 and r < 1.0:
        return "OK", "montage rigide et coaxial"
    if p < 0.15 and r < 3.0:
        return "PASSABLE", "leger flottement, utilisable avec filtrage"
    return "REFAIRE", ("rotation non planaire (planarity=%.3f, rms=%.1f deg) — "
                       "support qui flue, pignon desaxe, ou occlusion "
                       "pendant le balayage" % (p, r))


# ---------------------------------------------------------------- accumulation


class Accumulator:
    """Deroule theta pour supporter le multi-tour.

    Un decrochage optique long peut faire perdre un tour. Plutot que
    d'accumuler un saut faux en silence, on marque l'echantillon suspect :
    dropout() passe a True et le bridge l'affiche. C'est le mode de panne
    invisible que le handoff reproche aux trackers, rendu visible.
    """

    def __init__(self, jump_limit=math.radians(150.0)):
        self.total = 0.0
        self.prev = None
        self.jump_limit = jump_limit
        self._dropout = False

    def push(self, theta, gap=0.0):
        if self.prev is None:
            self.prev = theta
            self.total = theta
            return self.total

        d = theta - self.prev
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi

        # Saut enorme apres un trou de donnees : impossible de savoir si la
        # bague a fait un demi-tour ou un tour et demi.
        if gap > 0.25 and abs(d) > self.jump_limit:
            self._dropout = True

        self.total += d
        self.prev = theta
        return self.total

    def dropout(self):
        return self._dropout

    def clear_dropout(self):
        self._dropout = False


class OneEuro:
    """Filtre one-euro applique a theta — jamais au quaternion.

    Filtrer un quaternion composante par composante le denormalise et
    introduit une erreur d'angle dependante de l'orientation. Sur un scalaire
    deroule, le probleme ne se pose pas.

    Le bruit lighthouse n'est pas stationnaire : il depend du nombre de
    photodiodes vues a l'instant t. mincutoff bas lisse le statique, beta
    laisse passer les gestes francs de l'assistant camera.
    """

    def __init__(self, mincutoff=0.4, beta=0.015, dcutoff=1.0):
        self.mincutoff = mincutoff
        self.beta = beta
        self.dcutoff = dcutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.t_prev is None or t <= self.t_prev:
            self.x_prev, self.t_prev, self.dx_prev = x, t, 0.0
            return x
        dt = t - self.t_prev
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.dcutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.mincutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat


def to_freed(theta, lo, hi, full_scale=65535, invert=False):
    """theta deroule -> entier Free-D.

    full_scale reste a 65535 : c'est ce que --source simulate a valide a
    l'etape 1. Le passage en millimetres reels est le travail du LensFile.
    """
    if hi == lo:
        return 0
    v = (theta - lo) / (hi - lo)
    if invert:
        v = 1.0 - v
    return int(max(0, min(full_scale, round(v * full_scale))))


# ---------------------------------------------------------------- persistance


def load(path):
    with open(path) as f:
        return json.load(f)


def save(path, data):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


# ---------------------------------------------------------------- auto-test

if __name__ == "__main__":
    rng = np.random.default_rng(0)

    def q_from(axis, angle):
        a = np.array(axis, float)
        a /= np.linalg.norm(a)
        s = math.sin(angle / 2.0)
        return q_norm((math.cos(angle / 2.0), a[0] * s, a[1] * s, a[2] * s))

    # --- 1. Calibration : axe quelconque, montage de travers, 500 deg de
    #        course (multi-tour), swing de 0.3 deg injecte, bruit de mesure.
    true_axis = np.array([0.3, -0.85, 0.43])
    true_axis /= np.linalg.norm(true_axis)
    q_mount = q_from([1.0, 0.2, -0.4], 0.7)

    quats, truth = [], []
    for i in range(400):
        ang = math.radians(500.0) * i / 399.0
        swing = q_from(rng.normal(size=3), math.radians(0.3) * rng.normal())
        noise = q_from(rng.normal(size=3), math.radians(0.05) * rng.normal())
        quats.append(q_norm(q_mul(q_mount,
                                  q_mul(q_from(true_axis, ang),
                                        q_mul(swing, noise)))))
        truth.append(ang)

    cal = fit_axis(quats)
    err = math.degrees(math.acos(min(1.0, abs(float(np.dot(cal["axis"],
                                                           true_axis))))))
    v, _why = verdict(cal)
    print("[axe]      erreur %.3f deg | course %.1f deg | planarity %.4f | "
          "rms %.2f deg | %s"
          % (err, cal["span_deg"], cal["planarity"], cal["rms_deg"], v))

    inv_ref = q_conj(tuple(cal["ref"]))
    acc = Accumulator()
    worst = max(abs(math.degrees(acc.push(twist_angle(q_mul(inv_ref, q),
                                                      cal["axis"])) - t))
                for q, t in zip(quats, truth))
    print("[rejeu]    ecart max %.3f deg, pas de saut de tour" % worst)

    # --- 2. Soustraction du mouvement camera, echantillonnage asynchrone.
    #        Camera a 180 deg/s, tracker objectif horodate 4 ms apres la
    #        camera, bague STRICTEMENT IMMOBILE : theta doit rester constant.
    #
    #        La fuite depend de l'angle entre l'axe du mouvement camera et
    #        l'axe du pignon. Un pignon de follow focus tourne autour d'un axe
    #        parallele a l'axe optique : un PANORAMIQUE (autour de la
    #        verticale) est presque orthogonal et fuit peu, un ROULIS
    #        (dutch, epaule, steadicam) est colineaire et fuit en totalite.
    q_bague = q_mul(q_mount, q_from(true_axis, math.radians(120.0)))
    th_ref = twist_angle(q_mul(inv_ref, q_bague), cal["axis"])
    rate = math.radians(180.0)
    skew = 0.002          # objectif horodate 2 ms avant le dernier q_camera

    def async_run(cam_axis, skew):
        hist = CameraHistory()
        naive = aligned = 0.0
        for i in range(300):
            tc = i * 0.004                               # camera a 250 Hz
            q_cam = q_from(cam_axis, rate * tc)
            hist.push(tc, q_cam, (0.0, 0.0, 1.4))
            if i < 2:
                continue
            # L'echantillon objectif tombe ENTRE deux echantillons camera :
            # c'est le cas normal quand on vide la file a chaque tick. S'il
            # etait plus recent que le dernier q_camera, at() rendrait
            # "extrap" et le bridge doit alors differer d'un tick.
            tl = tc - skew
            q_lens = q_mul(q_from(cam_axis, rate * tl), q_bague)

            q_at, _p, state = hist.at(tl)
            # Seul "exact" est exploitable. "extrap" (echantillon objectif
            # plus recent que toute la camera) doit etre differe d'un tick ;
            # "stale" ne se produit qu'au demarrage. Les accepter reviendrait
            # a retomber sur le comportement naif.
            if state != "exact":
                continue
            th_n = twist_angle(q_mul(inv_ref, relative(q_cam, q_lens)),
                               cal["axis"])        # naif : dernier q_camera
            th_a = twist_angle(q_mul(inv_ref, relative(q_at, q_lens)),
                               cal["axis"])        # aligne : slerp a tl
            naive = max(naive, abs(math.degrees(th_n - th_ref)))
            aligned = max(aligned, abs(math.degrees(th_a - th_ref)))
        return naive, aligned, hist

    n_pan, a_pan, _ = async_run([0.0, 0.0, 1.0], skew)
    n_roll, a_roll, hist = async_run(cal["axis"], skew)
    # Meme scenario avec 20 ms d'ecart : une occlusion breve du tracker
    # objectif suffit a creuser ce trou, et l'erreur est proportionnelle.
    n_occ, a_occ, _ = async_run(cal["axis"], 0.020)

    print("[async]    pano   180 deg/s, ecart  2 ms : "
          "naif %.2f deg -> aligne %.3f deg" % (n_pan, a_pan))
    print("[async]    roulis 180 deg/s, ecart  2 ms : "
          "naif %.2f deg -> aligne %.3f deg" % (n_roll, a_roll))
    print("[async]    roulis 180 deg/s, ecart 20 ms : "
          "naif %.2f deg -> aligne %.3f deg" % (n_occ, a_occ))
    print("[cam rate] %.1f deg/s" % math.degrees(hist.rate()))

    # --- 3. Chien de garde du montage : glissement de 12 mm.
    mw = MountWatch(tolerance_mm=8.0)
    mw.push(relative_position((1, 0, 0, 0), (0, 0, 1.4), (0.150, 0.05, 1.4)))
    slipped = mw.push(relative_position((1, 0, 0, 0), (0, 0, 1.4),
                                        (0.162, 0.05, 1.4)))
    print("[montage]  glissement detecte : %s (derive %+.1f mm)"
          % (slipped, mw.drift * 1000))

    assert err < 0.5, err
    assert abs(cal["span_deg"] - 500.0) < 2.0, cal["span_deg"]
    assert worst < 2.0, worst
    assert n_occ > 2.0, n_occ                    # le probleme est bien reel
    assert a_occ < n_occ / 20.0, (n_occ, a_occ)
    assert a_roll < n_roll / 20.0, (n_roll, a_roll)
    assert a_pan < n_pan, (n_pan, a_pan)
    assert slipped
    assert to_freed(cal["lo"], cal["lo"], cal["hi"]) == 0
    assert to_freed(cal["hi"], cal["lo"], cal["hi"]) == 65535
    print("OK — axe, multi-tour, alignement temporel, chien de garde.")
