"""Five-motor PWM with an independent software heartbeat watchdog."""
import threading
import time
from core import motor_duties


class MotorDriver:
    def __init__(self, cfg, enabled=False, factory=None):
        self.cfg = cfg
        self.enabled = enabled
        self.lock = threading.Lock()
        self.motors = []
        self.duties = [0.0]*5
        self.heartbeat = time.monotonic()
        self.command_deadline = None
        self.stop = threading.Event()
        if enabled:
            if factory is None:
                from gpiozero import PWMOutputDevice
                from gpiozero.pins.lgpio import LGPIOFactory
                backend = LGPIOFactory()
                factory = lambda pin: PWMOutputDevice(pin, frequency=cfg['frequency_hz'], initial_value=0, pin_factory=backend)
                self.backend = backend
            try:
                for pin in cfg['pins']:
                    self.motors.append(factory(pin))
            except Exception:
                self.close()
                raise
        self.watchdog = threading.Thread(target=self._watch, daemon=True)
        self.watchdog.start()

    def _set(self, levels):
        self.duties = list(levels)
        for m, value in zip(self.motors, levels):
            m.value = value

    def apply_feedback(self, threats, valid_until=None):
        with self.lock:
            self._set(motor_duties(threats, self.cfg))
            self.heartbeat = time.monotonic()
            self.command_deadline = valid_until

    def clear(self):
        self.apply_feedback([])

    def _watch(self):
        while not self.stop.wait(0.025):
            try:
                with self.lock:
                    if (time.monotonic()-self.heartbeat>self.cfg['heartbeat_timeout_s'] or
                            self.command_deadline is not None and time.monotonic()>=self.command_deadline):
                        self._set([0.0]*5)
            except Exception as exc:
                print('[HAPTIC WATCHDOG]', exc, flush=True)
                self.stop.set()

    def close(self):
        success=True
        self.stop.set()
        watchdog = getattr(self, 'watchdog', None)
        if watchdog and watchdog is not threading.current_thread():
            watchdog.join(timeout=0.5)
        with self.lock:
            self.duties = [0.0]*5
            for m in self.motors:
                try:
                    m.value = 0
                except Exception as exc:
                    success=False
                    print('[HAPTIC CLEANUP] Zero command failed:', exc, flush=True)
                try:
                    m.close()
                except Exception as exc:
                    success=False
                    print('[HAPTIC CLEANUP] Device close failed:', exc, flush=True)
            self.motors = []
        if hasattr(self, 'backend'):
            self.backend.close()
        return success
