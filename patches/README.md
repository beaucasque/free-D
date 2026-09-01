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

**Amont.** Branche `submit/defer-usb-close` dans le clone, prête à pousser
sur un fork. Non soumise à ce jour.
