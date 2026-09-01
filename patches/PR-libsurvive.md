# Titre

driver_vive: fix two crashes in the USB transfer callback

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

## This is the first of two commits

Applying this one alone does **not** stop the abort. The re-entrancy it
removes is real, but the crash observed here comes from the second commit's
subject: `handle_transfer()` frees a transfer it has just resubmitted, so
`libusb_free_transfer()` destroys the mutex of a transfer still in the
flying-transfers list.

Confirmed the hard way: with only this commit applied, unplugging a running
tracker still aborted the process.

## Testing

Both commits, three wired trackers on a multi-TT hub: 238–242 Hz per device,
zero dropouts, lighthouse geometry solved as before, and unplugging a device
while running no longer aborts — the process keeps serving and reports the
device gone. Without them, it dies the instant the device goes away.
