# Titre

driver_vive: fix two crashes in the USB transfer callback

# Corps (à coller tel quel dans la description)

Two independent problems in `handle_transfer()`, libusb's
transfer-completion callback, both reached the moment a transfer fails — an
unplugged device, a long optical dropout, a tracker powering off, or bus
saturation.

The first is the crash I actually hit, every time:

```
Warning: 2.703381 T23 Device disconnect: 1
python: ../../libusb/os/threads_posix.h:46: usbi_mutex_lock:
        Assertion `pthread_mutex_lock(mutex) == 0' failed.
```

## A resubmitted transfer is then freed

Commit `do not free a resubmitted transfer`.

```c
if (iface->error_count++ < 10) {
    if (libusb_submit_transfer(transfer)) {   /* != 0 == failure */
        goto shutdown;
    }
}                    /* success falls out of the if */
goto disconnect;     /* ... and still reaches shutdown: */
```

When the resubmit **succeeds** the transfer is in flight again, yet control
falls through to the `shutdown:` label, which runs `libusb_free_transfer()`
on it. That destroys the transfer's mutex while libusb still has it in the
flying-transfers list; the next event-loop pass locks a destroyed mutex,
`pthread_mutex_lock()` fails, and `usbi_mutex_lock()` asserts.

Return after a successful resubmit, so the retry actually gets to run and
the in-flight transfer is left alone. Only a failed resubmit still goes to
`shutdown:`, which is correct — nothing is in flight then.

The same commit drops a duplicated increment: `error_count` was bumped twice
per error, so the retry budget was five failures rather than the ten the
code reads as.

## Re-entering libusb from inside its own callback

Commit `defer close out of the transfer callback`. This one I did not
observe aborting on its own; it is a re-entrancy that reading the code makes
plain, and it sits on the same failure path.

On a non-`COMPLETED` status, `survive_disconnect_device()` called
`survive_close_usb_device()` synchronously. That function calls
`libusb_cancel_transfer()` on every interface — including the one whose
callback is executing — and `survive_config_cancel()` reaches libusb the
same way. The event thread already holds that transfer's locks.

`survive_disconnect_device()` now only marks the interfaces down and raises
`request_disconnect`. The poll loop in `survive_usb_poll()` consumes it and
calls `survive_close_usb_device()` off-callback, where no libusb lock is
held. This mirrors the existing `request_close` mechanism, already consumed
from the same loop — the deferral scheme was there, this path simply did not
use it.

The transfer being handled is still cleaned up as before by the `shutdown:`
label, so per-transfer accounting (`active_transfers`, `request_close`) is
unchanged.

## How to reproduce

Simplest: run against wired trackers and unplug one. Before these commits
the process dies instantly.

Also reproducible without touching anything: four full-speed Vive trackers
(`28de:2300`) behind a **single-TT** USB 2.0 hub — here two cascaded Genesys
`05e3:0608`. Transaction-translator saturation makes a transfer fail about
2.7 s after start, every time. Three trackers on the same hub ran for hours
without a single error, which is why this can look like a device-count
problem. A multi-TT hub (Realtek `0bda:5411`) removes the trigger but not
the bugs.

```
cat /sys/bus/usb/devices/<hub>/bDeviceProtocol   # 02 == multi-TT
```

## Testing

Three wired trackers on a multi-TT hub, via pysurvive: 238–242 Hz per
device, zero dropouts, lighthouse geometry solved as before. Unplugging a
device while running no longer aborts — the process keeps running and
reports the device gone.

Worth stating plainly: the deferral commit **alone does not stop the abort**.
I wrote it first, believing it was the cause, then unplugged a tracker and
the process still died. Freeing the resubmitted transfer is what actually
kills it. Both are included because both are real, but only the second is
backed by an observed failure — the first is backed by reading the code.
