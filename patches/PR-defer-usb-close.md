# Titre

driver_vive: defer USB close out of the libusb transfer callback

# Corps

`handle_transfer()` is libusb's transfer-completion callback: it runs on the
event thread while libusb holds the current transfer's locks. On a
non-`COMPLETED` status it reached `survive_disconnect_device()`, which called
`survive_close_usb_device()` synchronously. That function calls
`libusb_cancel_transfer()` on every interface — including the one whose
callback is currently executing — and `survive_config_cancel()` reaches
libusb the same way.

Re-entering libusb from inside its own callback makes `pthread_mutex_lock()`
fail, so `usbi_mutex_lock()` (`os/threads_posix.h`) asserts and the process
aborts:

```
Warning: 2.703381 T23 Device disconnect: 1
python: ../../libusb/os/threads_posix.h:46: usbi_mutex_lock:
        Assertion `pthread_mutex_lock(mutex) == 0' failed.
```

## How to reproduce

Four full-speed Vive trackers (`28de:2300`) behind a **single-TT** USB 2.0
hub — here a pair of cascaded Genesys `05e3:0608`. Transaction-translator
saturation makes a transfer fail about 2.7 s after start, and the process
dies every time. Three trackers on the same hub ran for hours without a
single error. Moving to a multi-TT hub (Realtek `0bda:5411`) removes the
trigger but not the latent bug: any failing transfer reaches this path,
including a long optical dropout or a tracker powering off.

Checking a hub:

```
cat /sys/bus/usb/devices/<hub>/bDeviceProtocol   # 02 = multi-TT
```

## The change

`survive_disconnect_device()` now only marks the interfaces down and raises
`request_disconnect`. The poll loop in `survive_usb_poll()` consumes it and
calls `survive_close_usb_device()` off-callback, where no libusb lock is
held.

This mirrors the existing `request_close` mechanism, already consumed from
the same loop — the deferral scheme was there, this path simply did not use
it.

The transfer being handled is still cleaned up as before by the `shutdown:`
label at the end of `handle_transfer()`, so per-transfer accounting
(`active_transfers`, `request_close`) is unchanged.

## Testing

Built and run against three wired trackers plus one wireless-capable device
on a multi-TT hub: 237–242 Hz per device, zero dropouts, lighthouse geometry
solved as before. No behaviour change in the nominal path.

**Not yet verified against the trigger itself**: the multi-TT hub removed the
failing transfers, so the faulty path is no longer exercised on this rig. The
reasoning is from reading the call chain, and the nominal path is confirmed
free of regressions.

## Aside, not fixed here

`handle_transfer()` increments `iface->error_count` twice per error:

```c
iface->error_count++;
if (iface->error_count++ < 10) {
```

so the retry loop gives up after 5 failures rather than the 10 it reads as.
Left alone to keep this change to one concern.
