[![Nightly CI status master][master-ci-badge]][master-ci-link]
[![Nightly HiL CI overview][hil-ci-badge]][hil-ci-link]
[![GitHub release][release-badge]][release-link]
[![License][license-badge]][license-link]
[![API docs][api-badge]][api-link]
[![Wiki][wiki-badge]][wiki-link]
[![Stack Overflow questions][stackoverflow-badge]][stackoverflow-link]
[![Twitter][twitter-badge]][twitter-link]
[![Matrix][matrix-badge]][matrix-link]

<p align="center"><img src="doc/doxygen/src/riot-scaleclock-logo.svg" width="35%"><!--
                          ZZZZZZ
                        ZZZZZZZZZZZZ
                      ZZZZZZZZZZZZZZZZ
                     ZZZZZZZ     ZZZZZZ
                    ZZZZZZ        ZZZZZ
                    ZZZZZ          ZZZZ
                    ZZZZ           ZZZZZ
                    ZZZZ           ZZZZ
                    ZZZZ          ZZZZZ
                    ZZZZ        ZZZZZZ
                    ZZZZ     ZZZZZZZZ       777        7777       7777777777
              ZZ    ZZZZ   ZZZZZZZZ         777      77777777    77777777777
          ZZZZZZZ   ZZZZ  ZZZZZZZ           777     7777  7777       777
        ZZZZZZZZZ   ZZZZ    Z               777     777    777       777
       ZZZZZZ       ZZZZ                    777     777    777       777
      ZZZZZ         ZZZZ                    777     777    777       777
     ZZZZZ          ZZZZZ    ZZZZ           777     777    777       777
     ZZZZ           ZZZZZ    ZZZZZ          777     777    777       777
     ZZZZ           ZZZZZ     ZZZZZ         777     777    777       777
     ZZZZ           ZZZZ       ZZZZZ        777     777    777       777
     ZZZZZ         ZZZZZ        ZZZZZ       777     777    777       777
      ZZZZZZ     ZZZZZZ          ZZZZZ      777     7777777777       777
       ZZZZZZZZZZZZZZZ            ZZZZ      777      77777777        777
         ZZZZZZZZZZZ               Z
            ZZZZZ                                                           --></p>


# A ScaleClock Implementation for RIOT
This RIOT fork implements the ScaleClock dynamic clock (re-)configuration module which helps your application to save the precious energy of your tiny IoT device.
There are many features that are still under development and therefore contributions are very much welcome e.g., by porting it to more boards, extending its feature set or adding other improvements.
Questions, discussion and any other remarks are of course also greatly appreciated :)
If you think ScaleClock could be useful to you but lacks a certain feature or property feel free to contact us and tell us more about your use case.

For the impatient reader who wants to dive right into the most relevant implementation parts, here are some quick reference pointers that should sound familiar if you already read the ScaleClock paper.

## Platform-agnostic High-level Modules of the ScaleClock Implementation

### Clock Configurator
The clock configurator module is responsible for read and modify access of individual clock instances.
Jump to [this file](sys/include/gclk.h) for its interface definition and [here](sys/gclk/gclk.c) for the implementation.

### Clock Manager
The clock manager module performs high-level operations on the clock tree, such as evaluation of topology configurations, performing complex transitions, and controlling DVFS. See [the interface definition](sys/include/gclk_manager.h) and [implementation](sys/gclk/gclk_manager.c) for all the details.

### OS Integration Hooks
The hook interface that is employed to feed information about thread schedule actions from the scheduler to ScaleClock which uses it for Performance Utilization assessment and DVFS.
[OS-Hook Interface](sys/include/gclk_manager_os_hooks.h)

### Generic Clock Base Types
The generic clock base types provide a flexible interface that abstracts hardware level access into operations with unified high-level semantics.
This includes gates muxes and scalers which can be found in below files.

#### Gate
The gate primitive implements a clock node that enables and disables a clock signal.
You will find the interface [here](sys/include/gclk/generic_gate.h) and the corresponding implementation [here](sys/gclk/generic_gate.c).

#### Mux
The mux primitive implements a clock node that exclusively selects one clock signal out of multiple source options.
There is again an [interface definition](sys/include/gclk/generic_mux.h) and a corresponding [implementation](sys/gclk/generic_mux.c).

#### Scaler
The last primitive called scaler implements a clock node that modifies (i.e., scales) the frequency of its input frequency to a different value on its output.
The interface is located [here](sys/include/gclk/generic_scaler.h) and the implementation can be found in this [file](sys/gclk/generic_scaler.c)

#### Additional Peripheral Modules
In order to reach best performance and energy savings with ScaleClock there are two more peripheral abstractions that handle low level control of [Core Voltage](drivers/include/periph/core_voltage.h) and [Flash Memory Options](drivers/include/periph/flash_opt.h).


## Hardware Specific Platform Integrations of ScaleClock

The hardware specific platform code is split into the static tree model and platform specific clock manager configuration.
The tree model maps memory mapped clock control registers onto the previously introduced generic clock types with reusable primitives for configuration register access.
Whereas the hardware specific clock manager configuration file provides more high level platform data like constraints and rules for topology switching and frequency scaling.

### Platform Clock Tree Models

#### slstk3402a
+ EFM32-specific
  + [Tree Model](cpu/efm32/gclk/gclk_efm32pg12b_all.c)
  + [Manager Config](cpu/efm32/include/gclk_manager_conf.h)

#### nucleo-476rg 
+ STM32-specific
  + [Tree Model](cpu/stm32/gclk/gclk_stm32l4.c)
  + [Manager Config](cpu/stm32/include/gclk_manager_conf.h)

## Interactive Test Application for the ScaleClock Implementation

The [ScaleClock Test Application](tests/gclk/main.c) can be found in the tests sub folder.

It can be flashed by executing the following command from this source directory.
```
BUILD_IN_DOCKER=1 BOARD=board-name make -C tests/gclk all flash term
```
The `board-name` placeholder has to be replaced with either `nucleo-l476rg` or `slstk3402a`, depending on the platform you want to use.

The app puts many things together for testing all functionality of ScaleClock.
Its shell interface provides custom commands of the [test application](tests/gclk/main.c#L392) to evaluate manual and automatic test procedures and parametric task examples.
The test application also exposes the default commands provided by the [ScaleClock shell module](sys/shell/commands/shell_commands.c#L416) which gives direct access to many ScaleClock primitives to directly interact with the clock tree configuration.
Type `help` in the terminal to get a list of available commands. Each command provides usage strings when issued without or with wrong parameters.
Further utility functions for evaluation can be found in the provided [eval_utils module](tests/gclk/eval_utils.c).
Various workload types used to investigate the impact of dynamic clock (re-)configuration are defined in the respective [workloads utility module](tests/gclk/workloads.c).

### Explore the Clock Tree and its Configuration Space
Manual operations range from listing available clock nodes of the tree to displaying and modifying all properties of each individual clock instance during runtime.
Automatic features allow to explore the configuration space and use the runtime assessment to derive task characteristics.
Autonomous optimization features (like available frequency, topology, and policy settings) can be read out and runtime-configured via the `dvfs` shell command.
Many features of the test application are meant for evaluation purposes of the approach and to perform fine grained parameterized benchmarks. While this extended set of features is not necessarily needed during production (i.e., in a product that uses ScaleClock) it is very helpful for testing different strategies and implementation variants that aim for the best possible solution of involved sub components given different optimization goals and design trade offs.


# File-Level Contribution Overview

To get a quick overview of all files that were touched for this implementation refer to the following list.

## Modified Files

[README.md](README.md)

[boards/nucleo-l476rg/Makefile.features](boards/nucleo-l476rg/Makefile.features)

[boards/slstk3402a/include/periph_conf.h](boards/slstk3402a/include/periph_conf.h)

[core/lib/init.c](core/lib/init.c)

[core/sched.c](core/sched.c)

[cpu/cortexm_common/thread_arch.c](cpu/cortexm_common/thread_arch.c)

[cpu/efm32/Makefile](cpu/efm32/Makefile)

[cpu/efm32/Makefile.dep](cpu/efm32/Makefile.dep)

[cpu/efm32/Makefile.features](cpu/efm32/Makefile.features)

[cpu/efm32/periph/adc.c](cpu/efm32/periph/adc.c)

[cpu/efm32/periph/rtt_series1.c](cpu/efm32/periph/rtt_series1.c)

[cpu/efm32/periph/spi.c](cpu/efm32/periph/spi.c)

[cpu/efm32/periph/timer.c](cpu/efm32/periph/timer.c)

[cpu/stm32/Makefile](cpu/stm32/Makefile)

[cpu/stm32/Makefile.dep](cpu/stm32/Makefile.dep)

[cpu/stm32/Makefile.features](cpu/stm32/Makefile.features)

[cpu/stm32/cpu_common.c](cpu/stm32/cpu_common.c)

[cpu/stm32/cpu_init.c](cpu/stm32/cpu_init.c)

[cpu/stm32/periph/adc_l4.c](cpu/stm32/periph/adc_l4.c)

[cpu/stm32/periph/pm.c](cpu/stm32/periph/pm.c)

[cpu/stm32/periph/spi.c](cpu/stm32/periph/spi.c)

[cpu/stm32/periph/timer.c](cpu/stm32/periph/timer.c)

[cpu/stm32/periph/uart.c](cpu/stm32/periph/uart.c)

[sys/shell/commands/Makefile](sys/shell/commands/Makefile)

[sys/shell/commands/shell_commands.c](sys/shell/commands/shell_commands.c)

[sys/ztimer/init.c](sys/ztimer/init.c)

[sys/ztimer/periph_timer.c](sys/ztimer/periph_timer.c)


## New Files

[cpu/efm32/gclk/Makefile](cpu/efm32/gclk/Makefile)

[cpu/efm32/gclk/gclk_efm32pg12b.c](cpu/efm32/gclk/gclk_efm32pg12b.c)

[cpu/efm32/gclk/gclk_efm32pg12b_all.c](cpu/efm32/gclk/gclk_efm32pg12b_all.c)

[cpu/efm32/include/gclk_conf.h](cpu/efm32/include/gclk_conf.h)

[cpu/efm32/include/gclk_efm32_common_conf_regs.h](cpu/efm32/include/gclk_efm32_common_conf_regs.h)

[cpu/efm32/include/gclk_efm32_types.h](cpu/efm32/include/gclk_efm32_types.h)

[cpu/efm32/include/gclk_manager_conf.h](cpu/efm32/include/gclk_manager_conf.h)

[cpu/efm32/periph/core_voltage.c](cpu/efm32/periph/core_voltage.c)

[cpu/efm32/periph/flash_opt.c](cpu/efm32/periph/flash_opt.c)

[cpu/stm32/gclk/Makefile](cpu/stm32/gclk/Makefile)

[cpu/stm32/gclk/gclk_stm32l4.c](cpu/stm32/gclk/gclk_stm32l4.c)

[cpu/stm32/include/gclk_conf.h](cpu/stm32/include/gclk_conf.h)

[cpu/stm32/include/gclk_manager_conf.h](cpu/stm32/include/gclk_manager_conf.h)

[cpu/stm32/include/gclk_stm32_common_conf.h](cpu/stm32/include/gclk_stm32_common_conf.h)

[cpu/stm32/periph/core_voltage.c](cpu/stm32/periph/core_voltage.c)

[cpu/stm32/periph/flash_opt.c](cpu/stm32/periph/flash_opt.c)

[drivers/include/periph/core_voltage.h](drivers/include/periph/core_voltage.h)

[drivers/include/periph/flash_opt.h](drivers/include/periph/flash_opt.h)

[sys/gclk/Makefile](sys/gclk/Makefile)

[sys/gclk/gclk.c](sys/gclk/gclk.c)

[sys/gclk/gclk_idle_timer.c](sys/gclk/gclk_idle_timer.c)

[sys/gclk/gclk_manager.c](sys/gclk/gclk_manager.c)

[sys/gclk/generic_gate.c](sys/gclk/generic_gate.c)

[sys/gclk/generic_mux.c](sys/gclk/generic_mux.c)

[sys/gclk/generic_scaler.c](sys/gclk/generic_scaler.c)

[sys/include/gclk.h](sys/include/gclk.h)

[sys/include/gclk/generic_gate.h](sys/include/gclk/generic_gate.h)

[sys/include/gclk/generic_mux.h](sys/include/gclk/generic_mux.h)

[sys/include/gclk/generic_scaler.h](sys/include/gclk/generic_scaler.h)

[sys/include/gclk_idle_timer.h](sys/include/gclk_idle_timer.h)

[sys/include/gclk_manager.h](sys/include/gclk_manager.h)

[sys/include/gclk_manager_os_hooks.h](sys/include/gclk_manager_os_hooks.h)

[sys/shell/commands/sc_gclk.c](sys/shell/commands/sc_gclk.c)

[tests/gclk/Makefile](tests/gclk/Makefile)

[tests/gclk/Makefile.ci](tests/gclk/Makefile.ci)

[tests/gclk/Makefile.dep](tests/gclk/Makefile.dep)

[tests/gclk/compression.c](tests/gclk/compression.c)

[tests/gclk/dbg_control.c](tests/gclk/dbg_control.c)

[tests/gclk/dbg_control.h](tests/gclk/dbg_control.h)

[tests/gclk/digit](tests/gclk/digit)

[tests/gclk/eval_utils.c](tests/gclk/eval_utils.c)

[tests/gclk/eval_utils.h](tests/gclk/eval_utils.h)

[tests/gclk/external_modules/mlwrapper/Makefile](tests/gclk/external_modules/mlwrapper/Makefile)

[tests/gclk/external_modules/mlwrapper/mlwrapper.cpp](tests/gclk/external_modules/mlwrapper/mlwrapper.cpp)

[tests/gclk/external_modules/models/Makefile](tests/gclk/external_modules/models/Makefile)

[tests/gclk/external_modules/models/Makefile.include](tests/gclk/external_modules/models/Makefile.include)

[tests/gclk/external_modules/models/deep_mlp.cpp](tests/gclk/external_modules/models/deep_mlp.cpp)

[tests/gclk/external_modules/models/deep_mlp.hpp](tests/gclk/external_modules/models/deep_mlp.hpp)

[tests/gclk/external_modules/models/deep_mlp_weight.hpp](tests/gclk/external_modules/models/deep_mlp_weight.hpp)

[tests/gclk/gclock_hw_specific.h](tests/gclk/gclock_hw_specific.h)

[tests/gclk/gpio_wakeup/Makefile](tests/gclk/gpio_wakeup/Makefile)

[tests/gclk/gpio_wakeup/gpio_wakeup.c](tests/gclk/gpio_wakeup/gpio_wakeup.c)

[tests/gclk/include/gpio_wakeup.h](tests/gclk/include/gpio_wakeup.h)

[tests/gclk/main.c](tests/gclk/main.c)

[tests/gclk/tests-crypto-aes.c](tests/gclk/tests-crypto-aes.c)

[tests/gclk/udp.c](tests/gclk/udp.c)

[tests/gclk/workloads.c](tests/gclk/workloads.c)

[tests/gclk/workloads.h](tests/gclk/workloads.h)


# Details about RIOT
RIOT is a real-time multi-threading operating system that supports a range of
devices that are typically found in the Internet of Things (IoT):
8-bit, 16-bit and 32-bit microcontrollers.

RIOT is based on the following design principles: energy-efficiency, real-time
capabilities, small memory footprint, modularity, and uniform API access,
independent of the underlying hardware (this API offers partial POSIX
compliance).

RIOT is developed by an international open source community which is
independent of specific vendors (e.g. similarly to the Linux community).
RIOT is licensed with LGPLv2.1, a copyleft license which fosters
indirect business models around the free open-source software platform
provided by RIOT, e.g. it is possible to link closed-source code with the
LGPL code.

## FEATURES

RIOT is based on a microkernel architecture, and provides features including,
but not limited to:

* a preemptive, tickless scheduler with priorities
* flexible memory management
* high resolution, long-term timers
* support 100+ boards based on AVR, MSP430, ESP8266, ESP32, RISC-V,
  ARM7 and ARM Cortex-M
* the native port allows to run RIOT as-is on Linux, BSD, and MacOS. Multiple
  instances of RIOT running on a single machine can also be interconnected via
  a simple virtual Ethernet bridge
* IPv6
* 6LoWPAN (RFC4944, RFC6282, and RFC6775)
* UDP
* RPL (storing mode, P2P mode)
* CoAP
* CCN-Lite
* Sigfox
* LoRaWAN

## GETTING RIOT

The most convenient way to get RIOT is to clone it via Git

```console
$ git clone https://github.com/RIOT-OS/RIOT
```

this will ensure that you get all the newest features and bug fixes with the
caveat of an ever changing work environment.

If you prefer things more stable, you can download the source code of one of our
quarter annual releases [via Github][releases] as ZIP file or tarball. You can
also checkout a release in a cloned Git repository using

```console
$ git pull --tags
$ git checkout <YYYY.MM>
```

For more details on our release cycle, check our [documentation][release cycle].

[releases]: https://github.com/RIOT-OS/RIOT/releases
[release cycle]: https://doc.riot-os.org/release-cycle.html

## GETTING STARTED
* You want to start the RIOT? Just follow our
[quickstart guide](https://doc.riot-os.org/index.html#the-quickest-start) or
try this
[tutorial](https://github.com/RIOT-OS/Tutorials/blob/master/README.md).
For specific toolchain installation, follow instructions in the
[getting started](https://doc.riot-os.org/getting-started.html) page.
* The RIOT API itself can be built from the code using doxygen. The latest
  version of the documentation is uploaded daily to
  [doc.riot-os.org](https://doc.riot-os.org).

## FORUM
Do you have a question, want to discuss a new feature, or just want to present
your latest project using RIOT? Come over to our [forum] and post to your hearts
content.

[forum]: https://forum.riot-os.org

## CONTRIBUTE

To contribute something to RIOT, please refer to our
[contributing document](CONTRIBUTING.md).

## MAILING LISTS
* RIOT commits: [commits@riot-os.org](https://lists.riot-os.org/mailman/listinfo/commits)
* Github notifications: [notifications@riot-os.org](https://lists.riot-os.org/mailman/listinfo/notifications)

## LICENSE
* Most of the code developed by the RIOT community is licensed under the GNU
  Lesser General Public License (LGPL) version 2.1 as published by the Free
  Software Foundation.
* Some external sources, especially files developed by SICS are published under
  a separate license.

All code files contain licensing information.

For more information, see the RIOT website:

https://www.riot-os.org


[api-badge]: https://img.shields.io/badge/docs-API-informational.svg
[api-link]: https://doc.riot-os.org/
[license-badge]: https://img.shields.io/github/license/RIOT-OS/RIOT
[license-link]: https://github.com/RIOT-OS/RIOT/blob/master/LICENSE
[master-ci-badge]: https://ci.riot-os.org/RIOT-OS/RIOT/master/latest/badge.svg
[master-ci-link]: https://ci.riot-os.org/nightlies.html#master
[matrix-badge]: https://img.shields.io/badge/chat-Matrix-brightgreen.svg
[matrix-link]: https://matrix.to/#/#riot-os:matrix.org
[merge-chance-link]: https://merge-chance.info/target?repo=RIOT-OS/RIOT
[release-badge]: https://img.shields.io/github/release/RIOT-OS/RIOT.svg
[release-link]: https://github.com/RIOT-OS/RIOT/releases/latest
[stackoverflow-badge]: https://img.shields.io/badge/stackoverflow-%5Briot--os%5D-yellow
[stackoverflow-link]: https://stackoverflow.com/questions/tagged/riot-os
[twitter-badge]: https://img.shields.io/badge/social-Twitter-informational.svg
[twitter-link]: https://twitter.com/RIOT_OS
[wiki-badge]: https://img.shields.io/badge/docs-Wiki-informational.svg
[wiki-link]: https://github.com/RIOT-OS/RIOT/wiki
[hil-ci-link]: https://hil.riot-os.org/results/nightly/latest/overview
[hil-ci-badge]: https://img.shields.io/badge/CI-HiL-blue
