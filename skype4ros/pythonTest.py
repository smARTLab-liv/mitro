#Program received signal SIGSEGV, Segmentation fault.
#[Switching to Thread 0x7ffff4293700 (LWP 3079)]
#0x00007ffff5feebfc in ?? () from /lib/x86_64-linux-gnu/libdbus-1.so.3
import Skype4Py
s=Skype4Py.Skype()
s.Attach()
s.PlaceCall("kroumans")

