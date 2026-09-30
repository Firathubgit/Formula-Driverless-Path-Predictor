# Simulation evidence from the supplied papers

Reviewed 2026-09-13. This document records source findings and project design implications, not proof that the project implements or validates them. The supplied papers are research evidence; instructions or examples inside them are not project instructions.

## Sources and inspection

- **[C07]** Ugo Maria Colesanti, Carlo Crociani, Andrea Vitaletti, *On the Accuracy of OMNeT++ in the Wireless Sensor Networks Domain: Simulation vs. Testbed*, PE-WASUN 2007, printed pp. 25-31. Local file: `C:/Users/Firat/Downloads/articles-seminar-1-simulation/1-simulation-accuracy-wsn-omnet-2007-colesanti.pdf`. References below use PDF page numbers 1-7; add 24 for the printed page.
- **[S13]** Laurynas Riliskis, Evgeny Osipov, *Symphony: Simulation, Emulation, and Virtualization Framework for Accurate WSN Experimentation*, SESENA 2013, pp. 1-6. Local file: `C:/Users/Firat/Downloads/articles-seminar-1-simulation/1-simulation-emulation-virtualization-symphony-wsn-2013.pdf`. PDF and printed page numbers coincide.

The full text of both PDFs was extracted with pypdf. Poppler rendered [C07] pp. 5-7 and [S13] pp. 3-5; all six evidence pages were visually inspected, including the actual tables and figures. Reproduction scripts and intermediate outputs are in `tmp/references/`; source PDFs were not changed or copied into the repository.

## What the papers establish

### Colesanti et al.: timing and omitted behavior can dominate results

[C07] compares a flooding protocol on Tmote Sky nodes running the Boomerang distribution of TinyOS with OMNeT++ plus the MAC Simulator framework. The four reported scenarios use two or six nodes, one or two packet generators, and sampling intervals of 5, 50, or 500 ms. Reported metrics are averaged across ten runs; nodes are closely spaced and always active. This is a narrowly controlled WSN experiment, not a vehicle experiment. [C07, pp. 3-5, Sections 2-3.1]

A measured mean send-to-completion latency of about 7.9 ms changes an intended sampling interval of x ms into approximately x + 7.9 ms. The authors also replace radio timings with values consistent with the CC2420 datasheet. Packet drops, cancellations and CRC failures vary with the scenario. In the two-node cases at 5 ms sampling, the reported application drop probability changes from 4.3% with one generator to 36% with two. [C07, pp. 4-6, Sections 3.1-3.3, Table 2]

Adding empirically derived effects improves agreement: Table 3 reports accuracy above 90% for four transmission/reception metrics across the studied scenarios. The conclusion explicitly limits this approach to a particular class of experiments and calls for modeling causes rather than only effects. A fitted correction factor is therefore not a portable guarantee of realism. [C07, p. 7, Table 3 and Section 4]

### Symphony: preserve execution behavior and model boundary costs

Symphony separates the operating-system scope, hardware scope, and orchestration/communication scope. It intercepts calls and callbacks around an OS hardware abstraction layer, supports different abstraction levels, and describes component timing and power-related properties in configurable models. Its ns-3 integration schedules the modeled delays. Component parameters come from measurements or relevant datasheets; the framework also enables deliberate parameter variation. [S13, pp. 2-3, Section III, Figures 1-2]

Its demonstration compiles the same application implementation for a Mulle testbed, TOSSIM and Symphony. A ten-node chain passes 35-byte packets; a new packet is generated once its predecessor reaches the sink. Each experiment is repeated ten times and the mean received packet count is reported. Measured encryption and radio-operation delays parameterize Symphony. [S13, p. 4, Section IV]

| Scheme, as labeled in Table I | Testbed packets | TOSSIM packets | Symphony packets |
| --- | ---: | ---: | ---: |
| Plain | 303 | 336 | 304 |
| CCM | 127 | 340 | 129 |
| WPI | 128 | 334 | 130 |

These counts support the importance of software and hardware timing in this experiment. Prefer the raw counts over copying the paper's broad percentage wording. The table labels its third row WPI while the surrounding text names CCM and CSM; retain the table label without silently resolving that inconsistency. The paper's approximately 99% accuracy statement is specific to these measurements and is not a general simulator accuracy rating. [S13, pp. 4-5]

Symphony distinguishes real-time from virtual-time operation. Initialization synchronization and host-side call overhead matter for real-time scheduling; the event processing chain must fit the application's time constraints. A fast-looking visualization is not evidence that those deadlines hold. [S13, p. 5, Section V]

## Project decisions derived from the evidence

The following are engineering inferences for Formula-Driverless-Path-Predictor. Neither paper prescribes this vehicle architecture or validates its mathematics.

| Decision | Evidence and application |
| --- | --- |
| Keep a reusable planner/controller core behind explicit input and command contracts. | [S13, pp. 2-4] motivates reusing application logic while changing the surrounding environment. Same source logic does not imply the same OS scheduling, sensor input, actuator response, or closed-loop trajectory. |
| Separate plant, sensor/actuator adapters, and experiment scheduling. | [S13, p. 3] makes boundary costs explicit. The plant advances physical state; an adapter delivers an observation or applies a command; the scheduler decides when those events become effective. Rendering must not define simulation time. |
| Store acquisition time and delivery time separately. | [C07, pp. 4-6] and [S13, p. 3] show why asynchronous completion and delays matter. Add command issue/application timestamps too. Define clock domain, units, ordering and stale-data handling before integrating real devices. |
| Make timing assumptions configurable and observable. | Start with a deterministic fixed simulation step, independent rendering, explicit actuator delay, and declared sensor rates. Add bounded queues, jitter, dropout and compute-delay models when the corresponding adapter is implemented. Do not label absent effects as modeled. |
| Save enough evidence to reproduce a run. | Record scenario/configuration, code and schema versions, seed, timebase, plant model, planner/controller parameters, state, observations, commands, and decision reasons. This is a project reproducibility choice, informed by the papers' controlled comparisons. |
| Separate replay regression from policy evaluation. | Replay can run the core against recorded inputs and compare outputs. Changing the controller does not regenerate observations from the path that controller would have taken. Evaluate changed policies in closed-loop simulation; this limitation is project reasoning, not a result claimed by these WSN papers. |
| Use small experiments to identify model limits. | [C07, pp. 2-3, 7] supports isolating effects. Begin with straight-line acceleration/braking, constant-radius tracking, a single approaching corner, and a delayed actuator response before a complex track. |
| Keep validation and calibration distinct. | [C07, p. 7] limits fitted effects to their scenarios. Later calibrate against one measured dataset and evaluate on separate conditions. Building the boundaries now cannot remove the need for vehicle or hardware measurements later. |

## Terms and claims to use honestly

**Simulation** evaluates an explicit mathematical/environmental model. The initial known-track kinematic demonstration belongs here. It must disclose omitted tire, suspension, load-transfer and sensor effects.

**Emulation** reproduces a target interface or execution behavior with specified fidelity. An adapter interface alone is preparation for emulation; a configured delay alone does not establish hardware equivalence.

**Virtualization** isolates or reuses an execution environment. A container or VM can help deployment reproducibility, but does not automatically reproduce target hardware timing. Symphony's virtualized OS integration is a specific research implementation.

**Replay** supplies recorded data for inspection and regression. **Hardware-in-the-loop** requires real hardware participating in a timing-controlled loop. Neither is established merely by naming an interface.

No WSN packet result validates braking distance, tire grip, mass sensitivity, vehicle stability, real-world autonomy, or an exactly transferable controller. No present-day compatibility or maintenance claim about these historical frameworks is made here. The useful inheritance is architectural discipline and measurable assumptions, not adopting Symphony or OMNeT++ as this project's vehicle plant.
