rpi-rt-control
==============
Raspberry Pi Real-Time Control Support and Experiments
------------------------------------------------------

Repository with mainly educational level projects
to show how to use GNU/Linux in control applications.

Real-time task require controlled (limited) operating system
responses latencies. Standard Linux kernel does not guarantee
bounded latencies. That is why kernel with
[fully-preemptive patches](https://rt.wiki.kernel.org/index.php/Main_Page)
is requited. Real-time Linux support is tested and developed under
[OSADL.org](https://www.osadl.org/) organization coordination.

Raspberry Pi kernel sources with RT patches and Aufs patches applied
can be cloned from [linux-rpi](https://github.com/ppisa/linux-rpi)
repository.

The repository content:

 * [rpi_gpio_irc_module](kernel/modules/rpi_gpio_irc_module.c) -
   kernel driver implementation of quadrature/IRC sensor
   interface software only decoder. Sensor outputs are connected
   to GPIO pins.
 * [rpi_simple_dc_servo](appl/rpi_simple_dc_servo) -
   userspace DC motor control application which uses IRC driver
   and dirrect access to PWM and GPIO for direction output.
 * [rpi_dc_motor_control](simulink/rpi_dc_motor_control.slx) -
   the same application but implemented as Simulink model which
   uses C S-function implementation for PWM output and IRC
   driver read.

More information about project can be found in [InstallFest 2015 presentation slides](http://cmp.felk.cvut.cz/~pisa/installfest/rpi_overlay_and_rt.pdf).
[Lintarget](http://lintarget.sourceforge.net/) project the real RT-aware
Simulink ERT target is used for Simulink based experiment.
The article [Usable Simulink Embedded Coder Target for Linux](http://rtime.felk.cvut.cz/publications/public/ert_linux.pdf)
about Linux ERT has been presented at [16th Real Time Linux Workshop](https://www.osadl.org/Dusseldorf-2014.rtlws16-dusseldorf-2014.0.html).
Its use on Raspberry Pi platform is documented on next
[page](http://lintarget.sourceforge.net/rpi-motor-control/index.html).

Other related Raspberry Pi real-time demostration is control of
[CAN/CANopen stepper motors](http://pikron.com/pages/products/motion_control/sm_can.html)
equipped two joints arm from Simulink model. Simulink SocketCAN support
[is described on Lintarget page as well](http://lintarget.sourceforge.net/can_bus/index.html).
