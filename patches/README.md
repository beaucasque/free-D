# patches/

Correctifs appliqués à libsurvive avant compilation. `install.sh` repart
toujours d'`origin/master`, puis rejoue ce répertoire dans l'ordre
alphabétique. Il s'arrête franchement si l'un d'eux ne s'applique plus,
plutôt que de compiler du libsurvive non corrigé en silence.

Retirer un fichier d'ici dès que l'amont a intégré le correctif : la
prochaine exécution d'`install.sh` recompilera sans lui.

## 0001-libsurvive-deferred-usb-close.patch

**Ce qu'il corrige.** `handle_transfer()` est le callback de complétion de
libusb : il s'exécute sur le fil d'événements pendant que libusb tient les
verrous du transfert courant. Sur un statut différent de `COMPLETED`, il
atteignait `survive_disconnect_device()` → `survive_close_usb_device()`, qui
appelle `libusb_cancel_transfer()` sur **toutes** les interfaces, y compris
celle dont le callback s'exécute. Réentrer dans libusb depuis son propre
callback fait échouer `pthread_mutex_lock()`, et `usbi_mutex_lock()` lève son
assertion : `abort()`.

**Symptôme.** Reproduit avec quatre trackers full-speed derrière un hub USB
2.0 **single-TT** : la saturation du transaction translator fait échouer un
transfert ~2,7 s après le démarrage, et le processus meurt à chaque fois.

```
Warning: 2.703381 T23 Device disconnect: 1
python: ../../libusb/os/threads_posix.h:46: usbi_mutex_lock:
        Assertion `pthread_mutex_lock(mutex) == 0' failed.
```

Un hub multi-TT supprime le déclencheur, **pas** le défaut : tout transfert
en erreur emprunte ce chemin — décrochage optique prolongé, tracker qui
s'éteint.

**Comment.** `survive_disconnect_device()` ne fait plus que marquer les
interfaces et lever `request_disconnect` ; la boucle de poll le consomme et
ferme hors callback. C'est le schéma que libsurvive emploie déjà pour
`request_close`, consommé au même endroit : le mécanisme existait, ce
chemin-là ne l'utilisait pas.

**Amont.** Poussé le 1er septembre 2026 sur
`beaucasque/libsurvive`, branche `submit/no-free-inflight`. Pull request pas
encore ouverte. Voir `PR-body.md` (description) et `PR-title.txt` (titre).

**Attention :** ce correctif seul **ne suffit pas**. Il traite une réentrance
réelle mais secondaire ; la cause du plantage observé est dans le patch 0002.

## 0002-libsurvive-no-free-inflight-transfer.patch

**Ce qu'il corrige — c'est le correctif qui compte.** Dans
`handle_transfer()`, sur un statut différent de `COMPLETED` :

```c
if (iface->error_count++ < 10) {
    if (libusb_submit_transfer(transfer)) {   // != 0 = échec
        goto shutdown;
    }
}                    // succès : on sort du if
goto disconnect;     // et on tombe quand même dans shutdown:
```

Quand la resoumission **réussit**, le transfert repart en vol — puis le code
tombe dans `shutdown:`, qui appelle `libusb_free_transfer()` dessus. libusb
détruit son mutex alors qu'il figure encore dans la liste des transferts
actifs ; au passage suivant de la boucle d'événements, le verrou détruit fait
échouer `pthread_mutex_lock` et l'assertion tombe.

Il manquait un `return`.

Corrige au passage le double `error_count++` : le budget de reprise valait
cinq échecs au lieu des dix que le code annonce.

**Vérifié sur matériel** le 1er septembre 2026, contre le déclencheur :
tracker débranché en marche, `NRestarts` reste à 0, même PID de part et
d'autre, zéro assertion. Avant, le processus mourait à l'instant du
débranchement.

**Amont.** Branche `submit/no-free-inflight` sur
<https://github.com/beaucasque/libsurvive>, qui porte les deux correctifs.
Ouvrir la PR :
<https://github.com/beaucasque/libsurvive/pull/new/submit/no-free-inflight>,
en collant `PR-title.txt` dans le titre et `PR-body.md` dans la description.
