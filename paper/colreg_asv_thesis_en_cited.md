# Abstract

Autonomous surface vessels operating in harbours, port approaches, and constrained coastal waterways must do more than remain geometrically collision-free. They must also manoeuvre in ways that are timely, interpretable, and sufficiently aligned with the International Regulations for Preventing Collisions at Sea (COLREGs) that surrounding mariners can anticipate their behaviour. This dissertation develops, documents, and evaluates a rule-informed collision-avoidance pipeline implemented in Webots for a small autonomous surface vessel. The implemented system combines LiDAR point-cloud clustering, principal-component-based obstacle size estimation, multi-target tracking with an extended Kalman filter (EKF), closest-point-of-approach metrics, explicit encounter classification, and a size-aware artificial potential field (APF). The work is grounded in an earlier scoping study that proposed a broader architecture involving Behaviour Trees and GPMP2, but the present dissertation is intentionally restricted to the parts that are demonstrably implemented in code and supported by logs and figures.

The perception layer converts LiDAR returns into obstacle clusters using DBSCAN and estimates obstacle geometry through the first and second principal dimensions, denoted PC1 and PC2. These quantities are carried into the control layer through anisotropic, ellipse-based safety regions rather than a fixed circular influence distance. The motion layer uses EKF tracks to estimate relative target motion and to compute time to closest point of approach (TCPA) and distance at closest point of approach (DCPA). The decision layer classifies encounters as crossing, head-on, overtaking, or default obstacle interaction, then selects encounter-specific APF behaviour with lateral bias and side-locking consistent with the intended COLREG response. In addition, predicted future conflicts can be converted into virtual obstacles, allowing the controller to react before a target physically enters the immediate geometric danger region.

Evaluation is carried out in Webots across four encounter families: crossing from left to right, crossing from right to left, head-on approach, and overtaking. Each family is tested with both a large and a small target vessel and with four switch configurations: EKF on or off, and cluster-size-based APF scaling on or off. The reported results show that EKF-based prediction is the most important enabling mechanism in dynamic encounters. When prediction is disabled, the controller frequently degrades toward reactive static-obstacle treatment and its encounter-specific behaviour becomes less reliable. Size-aware APF scaling successfully separates large and small targets in three of the four paired scenarios, but the head-on case exhibits a reversal in PC1 ordering, indicating sensitivity to viewpoint, occlusion, and track association. Crossing and head-on cases show useful avoidance behaviour under some switch combinations, but overtaking remains the dominant failure mode across the presented runs.

The dissertation therefore does not claim full COLREG compliance. Instead, it makes a narrower and more defensible contribution: it demonstrates that an interpretable perception-prediction-rule-control pipeline can be implemented end to end, evaluated systematically in simulation, and analysed critically through trajectory, clearance, and obstacle-size evidence. The study further identifies the principal technical deficits that prevent stronger claims, including inconsistent success thresholds in post-processing, insufficient repeated-trial statistics, scenario-classification drift, and a weak overtaking strategy under near-collinear relative motion. These findings are significant because they reposition the project from a generic claim of “COLREG-compliant autonomy” toward a precise engineering statement of what has been achieved, what has been measured, and what remains to be improved before simulation evidence can support stronger maritime-safety claims.

Keywords: autonomous surface vessel; COLREGs; LiDAR clustering; extended Kalman filter; artificial potential field; Webots; collision avoidance; maritime autonomy

\pagebreak

[[TOC]]

\pagebreak

# 1 Introduction

## 1.1 Background and motivation

Autonomous surface vessels are no longer a speculative concept. They are increasingly used in environmental monitoring, harbour inspection, bathymetric survey, infrastructure observation, and logistics support, while larger-scale maritime autonomy is now a standing research and industrial agenda rather than a niche topic (Bae and Hong, 2023; Burmeister and Constapel, 2021). What makes maritime autonomy difficult is not simply the presence of obstacles. Ports, narrow waterways, and traffic separation zones contain moving vessels of different sizes, different manoeuvrability limits, and different legal responsibilities. In such spaces, an autonomous craft must decide not only whether another target is dangerous, but also what kind of encounter is unfolding and whether it should stand on, give way, or execute an avoidance action early enough to be legible to human operators (Hu et al., 2022; Chang et al., 2024).

This distinction is fundamental. Many collision-avoidance systems can reduce collision risk in a geometric sense, especially when the environment is sparse or when obstacles behave predictably. Maritime navigation, however, has a normative layer. COLREGs do not merely describe a shortest path around another vessel; they encode expectations about lookout, safe speed, assessment of collision risk, overtaking responsibility, crossing priority, and head-on response. The practical safety value of these rules lies partly in mutual predictability. Human mariners infer intention from course changes, timing, and passing side. An autonomous vessel that remains technically collision-free but manoeuvres too late, too erratically, or in the wrong direction can still behave unsafely in the operational sense captured by COLREGs (IMO, 1972; Hu et al., 2022).

The challenge intensifies when perception is noisy. Close-range maritime sensors must operate under partial observability, intermittent returns, clutter from infrastructure, and rapid geometry changes as targets approach, pass, and occlude one another. LiDAR provides dense range information and is well suited to harbour-scale interaction, but raw point clouds do not directly yield encounter categories or control commands. A controller therefore needs an intermediate chain: obstacle grouping, geometry estimation, state tracking, risk prediction, encounter classification, and then command generation. Each stage has its own assumptions and error modes, and the overall quality of the final manoeuvre depends on how those stages are coupled (Villa et al., 2020; Xie et al., 2024; Han et al., 2026).

The project reported in this dissertation addresses that coupling problem through an explicitly modular yet code-realistic architecture. Earlier scoping work proposed a broader stack centred on Behaviour Trees and GPMP2. That proposal was useful as a research design because it clarified the desired long-horizon integration of perception, rule selection, local reaction, and planned manoeuvring. But a dissertation must distinguish between design intent and implemented evidence. Inspection of the controller source, logs, and figures shows that the present code base already contains a substantive implemented chain consisting of LiDAR clustering, principal-dimension obstacle description, EKF-based target prediction, TCPA/DCPA risk estimation, encounter-aware APF routing, and comparative switch-based experiments. That implemented chain is the subject of this dissertation.

The practical reason for focusing on this narrower chain is that it reflects the actual state of the project. It is tempting in autonomous navigation research to overstate architectural intent: to describe a system by what it aims eventually to include rather than by what it currently does. Such overstatement is especially risky in maritime autonomy, where interpretability and verification matter as much as algorithmic novelty. This dissertation therefore adopts a conservative principle: only those claims that can be supported by controller code, simulation outputs, and reproducible figures are treated as present contributions. Behaviour Trees and GPMP2 remain relevant to the discussion, but they are handled as part of the motivating scoping rationale and future extension path, not as validated implemented modules.

## 1.2 Problem statement

The central problem is whether a small autonomous surface vessel, using only the currently implemented modules, can generate avoidance behaviour that is both practically safer and more interpretable than a purely reactive geometric controller when facing standard COLREG-type encounters. More specifically, the question is not “can the vessel avoid collision at least once in simulation?” but rather:

Can LiDAR clustering, obstacle-size estimation, EKF motion prediction, and an encounter-routed size-aware APF improve collision avoidance quality in typical crossing, head-on, and overtaking situations while preserving a decision structure that is inspectable and relatable to COLREG intent?

That question contains four sub-problems.

First, the vessel must transform raw point clouds into obstacle representations that are meaningful for navigation. A central theme of this project is that obstacle size matters. A controller that treats every target as a point or as a fixed-radius object ignores an important part of maritime semantics: passing a small craft and passing a large vessel do not require the same geometric margin or the same legible path shape (Szlapczynski and Szlapczynska, 2017).

Second, the vessel must estimate motion. Static obstacle repulsion is insufficient in a busy channel because the same target geometry can correspond to radically different future risk depending on its relative course and speed. Prediction is therefore needed not for elegance, but because collision avoidance in dynamic encounters is a fundamentally temporal task (Fiorini and Shiller, 1998; von Brandis et al., 2025; Wu et al., 2026).

Third, the vessel must map risk into encounter types. COLREG-relevant response depends on whether the interaction is a crossing, head-on, or overtaking case. A control law that ignores encounter identity may still avoid impact, but it cannot be said to behave in a rule-informed manner (IMO, 1972; Hu et al., 2022).

Fourth, the controller must generate actions that are robust to implementation constraints. The code in this project does not solve a large nonlinear optimization problem online. Instead, it uses a modified APF because APF is computationally light, explainable, and suitable for high-frequency local action (Khatib, 1986; Li et al., 2024a). The research question is therefore not whether APF is globally optimal; it is whether APF, when strengthened by size information, motion prediction, and rule-informed biasing, becomes useful enough to support meaningful autonomous encounter handling.

## 1.3 Research objectives

Derived from the scoping study but adapted to the implemented system, the dissertation pursues five operational objectives.

The first objective is to document a complete perception-to-control chain that can be traced from LiDAR observations to heading commands. The value here is not only functional performance but also engineering auditability.

The second objective is to evaluate whether obstacle size, represented by the principal dimensions PC1 and PC2 of clustered LiDAR targets, changes the resulting avoidance trajectories in a coherent manner across large-vessel and small-vessel variants of the same encounter.

The third objective is to determine whether EKF-based target prediction materially improves dynamic collision avoidance relative to purely reactive obstacle treatment. This is tested through switch-based ablation rather than through informal observation alone.

The fourth objective is to assess how well explicit encounter routing maps targets into crossing, head-on, and overtaking responses under the current implementation.

The fifth objective is to identify the system’s main failure modes and validation gaps, especially where qualitative trajectory success might be mistaken for stronger claims of COLREG compliance.

## 1.4 Dissertation contribution

The contribution of this dissertation is intentionally narrower than the language often used in autonomous maritime navigation papers, and that narrowness is a strength rather than a weakness.

First, the dissertation provides a code-grounded description of a working modular architecture that integrates LiDAR clustering, principal-dimension obstacle representation, EKF target tracking, CPA-based risk prediction, encounter-specific rule routing, and APF-based heading control in a single operational chain.

Second, it demonstrates how obstacle size can be inserted into a local reactive controller without abandoning interpretability. Instead of adding opaque learned latent states, the system uses observable geometric quantities, PC1 and PC2, to scale elliptical danger regions and passing-direction heuristics.

Third, it presents a structured ablation across four encounter families, two target sizes, and four switch combinations, thereby allowing the roles of prediction and size adaptation to be separated.

Fourth, and most importantly, the dissertation performs a critical interpretation of those results. It shows where the system appears promising, where it is fragile, and where the current evidence cannot support stronger claims. That last point matters because simulation-based maritime autonomy research often converges prematurely on compliance claims without harmonized evaluation standards (Cao et al., 2025; Chang et al., 2024).

## 1.5 Relationship to the scoping study

The scoping study for this MSc project described a future-oriented architecture in which Behaviour Trees would coordinate perception, local avoidance, and long-horizon Gaussian-process motion planning. That scoping document remains important for three reasons. It explains why the project was framed around busy waterways rather than open-water waypoint tracking; it highlights the importance of interpretable decision structures; and it motivates the need for both reactive and deliberative planning layers.

However, the implemented controller examined in this dissertation does not yet instantiate a formal Behaviour Tree or a GPMP2 planner. The actual code is based on direct routing between encounter-specific APF variants and uses EKF prediction and CPA logic to compensate for APF’s otherwise reactive nature. The scoping study is therefore adapted here as a design origin, not cited as empirical evidence. This distinction is essential for academic integrity. A scoping document can define intended architecture, planned work, and anticipated benefits; a dissertation must separate those intentions from achieved implementation.

Accordingly, references to Behaviour Trees and GPMP2 in this thesis serve two roles only: they frame the original planned research direction, and they help interpret how the current implementation could evolve. They do not appear in the results chapters as if they had been part of the tested control loop. This conservative treatment aligns the written thesis with the actual state of the controller and keeps the empirical argument defensible.

## 1.6 Thesis structure

Chapter 2 reviews the literature on COLREG formalization, autonomous maritime collision avoidance, LiDAR-based perception and tracking, reactive and predictive planning methods, and validation practice. Chapter 3 reconstructs the implemented system architecture from the controller source code and explains the perception, prediction, routing, and control components in detail. Chapter 4 describes the Webots experimental design, switch-based ablation structure, and the limitations of the evaluation protocol. Chapter 5 presents the results from the supplied figure sets. Chapter 6 discusses what those results imply about the value of EKF prediction, the role of size-aware obstacle modelling, the weakness of the overtaking behaviour, and the gap between local geometric success and broader COLREG compliance. Chapter 7 concludes and proposes a practical future-work path toward stronger validation and more mature navigation architecture.

\pagebreak

# 2 Literature Review

## 2.1 COLREGs as a machine-interpretable decision framework

The International Regulations for Preventing Collisions at Sea were written for navigators, not for autonomous control software. Their practical strength lies in combining geometry, seamanship, and social expectation. Rules 5 to 8 address lookout, safe speed, assessment of collision risk, and avoidance action; Rules 13 to 17 specify responsibilities in overtaking, head-on, and crossing encounters, as well as the behaviour expected from stand-on and give-way vessels (IMO, 1972). For autonomous navigation research, the problem is therefore not whether COLREGs exist, but how to turn human-language rules into machine-executable conditions without stripping away the intended semantics.

Recent reviews agree that this translation remains incomplete across the field. Hu et al. (2022) show that most autonomous surface vehicle studies focus on a subset of encounter types, typically overtaking, head-on, and crossing, and often reduce COLREG compliance to heuristic geometry constraints. Burmeister and Constapel (2021) make a similar point from the perspective of maritime autonomy at sea: the state of the art contains many useful local mechanisms, but only a minority of systems provide transparent logic connecting perception to rule-specific action. Chang et al. (2024) extend this argument by identifying a wider gap between COLREG interpretation and the validation practices used to claim compliance. Across these studies, one message recurs: collision avoidance is not equivalent to rule compliance, and rule compliance is not equivalent to legal or operational acceptability.

This matters for system design. If an autonomous craft turns to starboard in a head-on encounter, it may appear compliant. But if it delays too long, oscillates, or crosses through a dangerous region before the turn becomes visible, its behaviour may still violate the requirement for positive action made in ample time in Rule 8. Likewise, a crossing encounter cannot be treated simply as “obstacle on the right means turn left or right according to cost”. The give-way relationship depends on relative bearing, relative motion, and the expectation that manoeuvres be clear and not indecisive (IMO, 1972; Kuwata et al., 2014; Perera et al., 2012). The implementation challenge is therefore twofold: first, classify the encounter; second, preserve a chosen response long enough that the action remains legible.

Figure 1 condenses the four encounter responsibilities used throughout this dissertation. It is a conceptual schematic rather than a geometric decision boundary: the exact trigger sectors and thresholds remain implementation choices, while the responsibilities and initial manoeuvre expectations derive from Rules 13-17 (IMO, 1972).

[[FIGURE:F:/Desk/IP_test5/paper/assets/colreg_encounter_schematic.png|Figure 1. Conceptual COLREG encounter schematic for head-on, crossing give-way, crossing stand-on, and overtaking situations, adapted from the responsibilities in Rules 13-17 (IMO, 1972).]]

## 2.2 Maritime collision-avoidance architectures

The collision-avoidance literature for maritime autonomy is structurally diverse but conceptually repetitive. Most systems combine some mixture of risk detection, encounter reasoning, path generation, and low-level control. Zhang et al. (2021) describe this as a broad state-of-the-art split between reactive local methods, optimization-based planners, and decision frameworks. Xu and Guedes Soares (2023) review path-following control for maritime autonomous surface ships and note that guidance and collision avoidance are often treated separately even though operationally they are tightly coupled. Cao et al. (2025), focusing on testing and evaluation of intelligent vessel collision avoidance, argue that the absence of common performance standards makes it hard to compare architectures that make very different assumptions about sensing, planning horizon, and vehicle dynamics.

One way to interpret this landscape is by the timescale on which methods act. Reactive methods such as artificial potential fields and the dynamic window approach operate at short horizons and prioritize immediate feasibility (Khatib, 1986; Fox et al., 1997). Predictive optimization and deterministic trajectory-planning methods reason over longer horizons and can incorporate vehicle constraints more explicitly (Lazarowska, 2017; Zhang et al., 2022). Rule-routing or symbolic methods sit above both levels and decide which behavioural mode should govern the encounter (Iovino et al., 2022). No single layer is sufficient by itself in busy waterways. A purely reactive controller may avoid impact locally while failing to produce consistent rule-based behaviour. A purely deliberative planner may generate a smooth path but struggle with fast updates or with imperfect target-state information. A purely symbolic rule engine still needs a guidance and motion-control layer capable of turning rule intent into feasible control (Fossen, 2011; Breivik and Fossen, 2009).

This dissertation sits in the middle of that spectrum. The present controller is reactive at the action-generation level, predictive at the target-risk level, and symbolic at the encounter-selection level. That combination is pragmatic: it accepts that a small Webots vessel controller benefits from a light computational footprint, but it also refuses to reduce dynamic encounters to static repulsion.

## 2.3 Artificial potential fields in maritime avoidance

Artificial potential fields remain attractive in maritime robotics because they are simple, cheap, and interpretable. Khatib’s classic formulation for obstacle avoidance in robotics established the idea of composing attractive and repulsive influences in configuration space so that the system moves along a resultant field gradient (Khatib, 1986). In maritime applications, this formulation is attractive because an own-ship trajectory can be biased toward a route or goal while obstacles contribute repulsive terms whose shapes and magnitudes can be tuned. APF-based strategies can therefore run at high control rates and respond smoothly to local changes in the scene.

However, the limitations of APF are equally well known. Standard potential fields can fall into local minima, oscillate near constraints, or produce unstable behaviour when the geometry of multiple obstacles creates competing gradients (Khatib, 1986). In maritime settings, these issues are amplified by encounter semantics. If the field simply pushes away from occupied geometry, it has no intrinsic understanding of whether a starboard-side target should be passed astern or whether a head-on encounter should induce a starboard alteration. Lyu et al. (2024) address this by embedding precise potential-field modelling and COLREG constraints in complex sailing environments. Li et al. (2024a) incorporate event-triggered control and artificial potential fields into a collision-avoidance decision-making scheme. These studies suggest that APF becomes more relevant to maritime autonomy when it is no longer a purely generic obstacle repulsion mechanism, but instead absorbs motion, rule, and passage-side information.

The present project follows that direction. It does not attempt to prove that APF is globally best. It uses APF because the controller already implements it, because its force decomposition is easy to log and interpret, and because it provides a transparent surface on which size- and rule-aware modifications can be layered. In that sense, the dissertation is not a generic defence of APF; it is a study of whether APF becomes practically stronger when fed by better perception and more explicit encounter logic.

## 2.4 Velocity obstacles and velocity-space reasoning

Velocity-obstacle methods occupy a different conceptual position. Instead of describing danger directly in physical space, they map unsafe future interactions into forbidden regions of the own-ship velocity space. Fiorini and Shiller (1998) provided one of the foundational formulations by defining the set of velocities that would lead to collision under relative motion assumptions. In maritime research, this approach is attractive because the collision problem is inherently temporal, and velocity-space reasoning can naturally represent closing trajectories.

Experimental and analytical maritime extensions confirm the value of this idea. Kuwata et al. (2014) integrated COLREG considerations with velocity obstacles for safe maritime autonomous navigation. Cho et al. (2019) experimentally validated a velocity-obstacle-based collision avoidance algorithm for unmanned surface vehicles, showing the method’s practical relevance in dynamic conditions. Thyri and Breivik (2022) developed partly COLREGs-compliant collision avoidance for ASVs using encounter-specific velocity obstacles, making explicit how velocity-space reasoning can be constrained by rule interpretation. Yu and Roh (2024) combined velocity obstacles with A* for anti-collision path planning in maritime autonomous surface ships, while Cao et al. (2025b) proposed a velocity-obstacle-based decision-making system for constrained waterways.

The benefit of velocity-space methods is their explicit treatment of relative motion. Their weakness is their dependence on reliable state estimation and on sufficient feasible velocity alternatives. In dense traffic or confined waterways, the safe velocity set can become thin, fragmented, or highly sensitive to prediction error. This is one reason why the current dissertation does not present velocity obstacles as a direct replacement for the implemented APF system. Instead, it treats the VO literature as evidence that motion prediction and encounter-specific directionality matter. The current controller expresses those ideas not by solving directly in velocity space, but by embedding prediction and side-specific bias into the APF logic.

## 2.5 Predictive optimization: MPC and GPMP2

Model predictive control is a major reference point for autonomous collision avoidance because it formulates navigation as repeated finite-horizon optimization under dynamic and safety constraints. Zhang et al. (2022) demonstrate time-optimal obstacle avoidance for autonomous ships using nonlinear MPC, while Zhang et al. (2025) study nonlinear MPC for path following in confined waterways. The strength of MPC lies in its ability to enforce actuator limits, vessel dynamics, and collision constraints in a single optimization problem. The weakness lies in the need for accurate models, careful tuning, and enough computational budget to re-solve the optimization at operational timescales.

Gaussian-process motion planning offers a different long-horizon perspective. Mukadam et al. (2018) showed how continuous-time Gaussian-process motion planning can be formulated as probabilistic inference. Meng et al. (2022) later adapted these ideas to autonomous surface vessels in environments with ocean currents through anisotropic GPMP2. The attraction of GPMP2 in the original scoping study was therefore sensible: it promises smooth trajectories, explicit uncertainty treatment, and a more global planning horizon than pure APF.

Yet those strengths do not remove implementation burden. GPMP2 requires a substantial planning layer beyond what is present in the current codebase. For this dissertation, the value of the GPMP2 literature is twofold. First, it justifies why the scoping study aimed beyond purely local reactive control. Second, it establishes a plausible future extension path for the present controller once the underlying perception and encounter logic are reliable enough to justify a larger planning stack.

## 2.6 Behaviour Trees and interpretable decision architectures

One of the more important ideas in the scoping study was that autonomous maritime navigation should be inspectable. This is why Behaviour Trees were proposed as the top-level coordination structure. Colledanchise and Ögren (2018) and Iovino et al. (2022) describe Behaviour Trees as modular, hierarchical control structures that scale better than finite-state machines when behaviour complexity grows. Their value is not merely software cleanliness. In safety-relevant applications they make the link between conditions and chosen actions easier to inspect, modify, and explain.

That interpretability argument is highly relevant to maritime autonomy. Alamoush and Ölçer (2025) discuss broader architecture issues in maritime autonomous surface ships, while Merino-Fidalgo et al. (2025), though not a maritime paper, illustrate how behaviour-tree generation and adaptation support interpretable robotic control. In the maritime domain, the main appeal of BTs is that they can express sequences such as “detect target”, “assess risk”, “classify encounter”, “apply give-way behaviour”, and “recover to route” in a formal control structure.

The current implementation stops short of a formal BT. Nevertheless, the explicit rule-routing logic in the controller already behaves like a proto-BT in engineering terms: it evaluates conditions, selects a controller profile, and preserves encounter-specific decision state. This matters because the present work is not conceptually disconnected from the scoping study. Rather, it can be interpreted as a lower-level precursor that already exhibits comparable transparency goals, albeit without the full BT formalism.

## 2.7 LiDAR perception, clustering, and target tracking

Perception is central to this dissertation because the system’s rule logic is only as good as the obstacle representations it receives. LiDAR is particularly suitable for harbour and near-range autonomy because it provides high spatial resolution, direct geometry, and independence from visual texture. Villa et al. (2020) show the usefulness of LiDAR-based obstacle avoidance for unmanned surface vehicles in harbour conditions. Xie, Nanlal, and Liu (2024) provide a more direct reference to the current project by studying reliable LiDAR-based ship detection and tracking for autonomous surface vehicles in busy maritime environments. Han et al. (2026) broaden the discussion through a survey of maritime perception datasets and technologies for autonomous surface vessels.

Raw LiDAR points must be transformed into navigational objects. DBSCAN is an attractive clustering choice because it does not require pre-specifying the number of targets and can separate sparse noise from denser structures (Ester et al., 1996). In marine settings, however, the point cloud of a vessel is not stable in the way a static landmark may be. Returns depend on angle, distance, occlusion, reflective surfaces, water clutter, and target pose. This means that clustering is not merely a preprocessing convenience. It directly shapes the geometry on which the avoidance controller operates.

Tracking then extends perception through time. The Kalman-filter family remains a standard engineering choice because it offers a computationally light way to combine motion prediction and noisy measurements. Kalman’s original 1960 paper established the linear filtering framework that later extensions generalized to nonlinear state-estimation problems (Kalman, 1960). The extended Kalman filter is not novel in itself, but its role in this project is critical: it converts transient cluster positions into predicted relative motion, which then supports CPA estimation and virtual obstacle generation.

Sensor fusion work also reinforces this direction. Von Brandis, Menges, and Rasheed (2025) combine LiDAR and AIS for multi-target tracking of autonomous surface vessels, highlighting the advantages of fusing geometric and identity-bearing measurements. The present project does not implement AIS fusion, but the literature helps interpret one of its main limitations: when only LiDAR clusters are available, target identity persistence and geometric stability become much more fragile.

## 2.8 Collision-risk metrics and vessel trajectory prediction

Collision avoidance depends not just on where targets are, but on what they are likely to do. The most common summary variables for such reasoning remain TCPA and DCPA. These metrics do not solve collision avoidance by themselves, but they offer a compact way to distinguish an obstacle that is nearby yet diverging from one that is currently farther away but on a converging course. Their appeal is especially strong in rule-based systems because they are interpretable, cheap to compute, and easy to log (Tam and Bucknall, 2010; Mou et al., 2010).

Trajectory-prediction literature supports the same principle at a broader scale. Li, Yu, and Yang (2024b) propose a hybrid vessel trajectory prediction methodology aimed at enhanced maritime navigation safety. Wu et al. (2026) address uncertainty explicitly in optimization methods for collision avoidance paths of unmanned vessels under ship-position prediction uncertainty. Even when such methods are more advanced than the current controller, their relevance here is direct: they show that dynamic encounter handling depends on state prediction fidelity, not only on geometric repulsion.

This project uses a deliberately compact constant-turn-rate-and-velocity (CTRV) motion model inside an EKF update loop. The state can represent straight motion through the zero-turn-rate limit and curved motion through a bounded turn rate, but it does not estimate acceleration as an independent state. This is appropriate to the implemented scope while still defining a limitation: abrupt acceleration and non-uniform manoeuvres can remain poorly predicted. The literature therefore helps interpret both the usefulness and the incompleteness of the present prediction layer (Kalman, 1960; Xie et al., 2024).

## 2.9 Validation gaps in the literature

Perhaps the most important insight from the literature is methodological rather than algorithmic. Cao et al. (2025a) review collision-avoidance performance testing and emphasise the lack of harmonized evaluation procedures. Chang et al. (2024) likewise point to gaps in how research trends and validation claims are connected in the MASS literature. This is particularly relevant to simulation studies. Reviews and traffic-risk studies show that evaluation assumptions, encounter generation, and validation environments vary substantially, while common benchmarks remain limited (Statheros et al., 2008; Goerlandt and Kujala, 2011; Burmeister and Constapel, 2021). Many papers evaluate a small number of canonical encounters and report visually successful trajectories, but fewer papers provide repeated trials, uncertainty perturbations, clear success thresholds, or separation between rule correctness and geometric non-collision.

This dissertation inherits exactly that validation problem. The supplied experiments are rich enough to support comparative insight, but they are not a statistically repeated campaign. Moreover, the already-generated figures expose threshold inconsistencies in how “success” was labelled across scenario families. Rather than concealing that issue, the dissertation adopts it as part of the critical analysis. A methodologically honest thesis must report not only what the controller did well, but also where the evaluation protocol itself was internally inconsistent.

## 2.10 Literature synthesis

Taken together, the literature supports a clear design position for this dissertation.

First, COLREG-informed autonomy benefits from explicit and interpretable decision structures rather than from purely black-box policies when the aim is to analyse behaviour and engineering trade-offs (Hu et al., 2022; Iovino et al., 2022).

Second, local geometric methods such as APF remain valuable in real-time control, but they become more useful when enriched by encounter semantics, obstacle shape, and motion prediction (Khatib, 1986; Lyu et al., 2024; Li et al., 2024a).

Third, dynamic collision avoidance requires state prediction. Whether one uses VO, MPC, or lighter EKF-plus-CPA logic, the literature consistently shows that relative motion matters (Cho et al., 2019; Thyri and Breivik, 2022; Zhang et al., 2022).

Fourth, perception quality and target representation are not secondary details. In maritime clutter, object identity and geometric extent strongly affect downstream control (Villa et al., 2020; Xie et al., 2024; von Brandis et al., 2025).

Fifth, the field still lacks robust validation norms. This means that a valuable MSc contribution need not be a universally superior controller. It can instead be a tightly scoped implementation study that clearly separates what is working, what is plausible, and what remains unproven. That is the stance adopted in the remainder of this thesis.

\pagebreak

# 3 System Architecture and Methodology

## 3.1 Reconstructing the implemented architecture

The controller examined in this dissertation is defined primarily in `src/laptop.py`, which acts as the unifying entry point across previously separate crossing and overtaking/head-on strategy files. This architectural detail matters because the project history included multiple scenario-specific controllers. The unified file consolidates switch combinations, shared APF parameters, scenario dispatch, logging, and obstacle snapshot writing into a single operational path. In practice, this means the system can be interpreted as a four-layer pipeline:

1. LiDAR perception and clustering generate obstacle candidates.
2. Obstacle geometry and tracking produce stateful target representations.
3. Encounter and risk logic select a rule-relevant controller profile.
4. APF control converts the selected profile into heading-oriented action.

The implementation is not a formal middleware stack in the ROS sense, but it is modular enough to audit. Shared switch combinations control whether obstacle EKF prediction is enabled and whether cluster-size-based APF scaling is enabled. A fixed classic APF influence distance is retained as a fallback when size scaling is disabled. The code explicitly copies or synchronizes functions from the crossing and overtaking controllers into the unified controller namespace, which reveals the project’s evolutionary development path: what is now a single controller originated as separate scenario-specific solutions.

This is important for interpretation. It explains why the dissertation must discuss architecture in engineering rather than idealized terms. The current system is neither a clean-slate planner nor a symbolic reasoning engine built from first principles. It is a practical, testable unification of earlier scenario controllers into a common APF and EKF framework. That pragmatism is one of its strengths because it exposes exactly which components were necessary to get from simulation sensing to navigation action.

## 3.2 Operating modes and experimental switch design

The unified controller contains four explicit switch combinations:

`ekf_on_cluster_on`, `ekf_on_cluster_off`, `ekf_off_cluster_on`, and `ekf_off_cluster_off`.

These switches are not a superficial convenience. They define the core ablation design of the dissertation. With EKF on, the controller can assign target tracks, estimate relative velocities, compute CPA metrics from state estimates, and generate dynamic virtual obstacles. With EKF off, targets can still exist geometrically, but they do not contribute the same predictive structure. The controller then behaves much more like a reactive obstacle-avoidance system.

With cluster-size APF range enabled, obstacle geometry from PC1 and PC2 influences the shape and scale of the obstacle safety field. With the switch off, the controller falls back to a classic fixed-range APF treatment. These two toggles isolate two distinct research variables:

prediction awareness and geometry awareness.

The switch design is one of the cleanest aspects of the implementation because it directly links architecture to evaluation. Instead of comparing unrelated controller families, the experiments compare the same controller under targeted removals of capability.

## 3.3 LiDAR clustering and obstacle representation

The first operational stage is clustering LiDAR returns into obstacle groups. The codebase imports a `cluster_principal_dimensions` helper and associated ellipse-level functions from the crossing controller module, indicating that obstacle geometry is part of the common APF infrastructure rather than a visualization-only post-processing step. Conceptually, the LiDAR pipeline transforms raw returns from the own vessel frame into clusters that are then described through:

- cluster centre position;
- principal orientation;
- first principal dimension, PC1;
- second principal dimension, PC2;
- an equivalent obstacle radius based on those dimensions.

The use of principal dimensions is significant. A conventional local planner might store only obstacle centroids and maybe a circular radius. Here, PC1 and PC2 act as proxies for target length and width in the observed point cloud. Because LiDAR sees only the surfaces available from the current viewpoint, these values are not true vessel dimensions. They are observed geometric extents. Yet that observed extent is still operationally useful because it changes the spatial interpretation of risk. A target with a long PC1 but narrow PC2 should not influence the local field in the same way as a compact target, even if their centroids are at the same distance.

The code includes minimum values for PC1 and PC2 to prevent degenerate clusters from collapsing the APF representation. This reveals an important engineering choice: the system assumes that very small observed extents are more likely to reflect under-sampling than genuinely tiny navigational hazards. In other words, the controller biases toward conservative geometry floors rather than trusting all cluster measurements equally.

## 3.4 Principal-component size estimation

The current project uses principal-component geometry not for object classification in the machine-learning sense, but as an engineering descriptor of obstacle scale. After clustering, the target point set is projected onto its dominant axes and the extents along those axes define PC1 and PC2. The first principal dimension represents the dominant elongation of the observed target point set, while PC2 captures its secondary width.

This matters for the APF because the controller treats obstacle danger as anisotropic rather than isotropic. A maritime target is rarely well approximated by a point. Even if the true hull outline is not reconstructed, an elliptical proxy based on principal dimensions is already a meaningful improvement over a fixed circular keep-out region. It supports at least three behaviours:

first, different safety extents for large and small targets; second, different effective approach risk depending on whether the own vessel is closing toward a target’s long or short axis; third, more realistic preferred passing direction when a route should go around the bow or stern of a target-like object rather than merely around its centroid.

The size signal is also filtered through time by track association. This is important because raw frame-wise PC1 and PC2 values are noisy. A single LiDAR frame can under- or over-estimate extent because of occlusion, grazing angle, or partial observation. The controller therefore does not rely on a single observation in isolation when a stable track exists.

## 3.5 Track management and extended Kalman filtering

The controller’s tracking logic is where the system transitions from geometric detection to dynamic understanding. Track association is controlled by a distance gate, and the code defines operating-mode-specific association and timeout parameters. Once a target cluster is linked to a track, the EKF predicts and updates its state over time. The dissertation does not rely on a custom novel filter design. The value of the EKF here lies in its role within the larger navigation architecture.

The practical function of the EKF is threefold.

First, it turns frame-to-frame cluster positions into estimated obstacle velocities. This is required for dynamic encounter classification and CPA computation.

Second, it stabilizes target identity. Without tracking, a target can flicker between observed positions and the controller may treat each observation as a new obstacle. In a crowded or partially occluded environment, that would make decision continuity extremely weak.

Third, the EKF supplies a basis for virtual future obstacles. A predicted target state can be projected forward over the same horizon as the own vessel, allowing risk to be acted on before physical proximity alone would justify strong local repulsion.

The filter follows the standard nonlinear prediction-correction structure derived from Kalman filtering (Kalman, 1960). The implemented obstacle state is

[[EQUATION:xₖ = [pₓ,ₖ, pᵧ,ₖ, vₖ, φₖ, ωₖ]ᵀ|(3.1)]]

where pₓ and pᵧ are target position in the north-east frame, v is speed, φ is heading, and ω is turn rate. For a sampling interval Δt, the CTRV transition used in `ObstacleEKF._ctrv_step()` is

[[EQUATION:pₓ,ₖ₊₁ = pₓ,ₖ + (vₖ/ωₖ)[sin(φₖ + ωₖΔt) − sin φₖ]|(3.2)]]

[[EQUATION:pᵧ,ₖ₊₁ = pᵧ,ₖ + (vₖ/ωₖ)[−cos(φₖ + ωₖΔt) + cos φₖ]|(3.3)]]

[[EQUATION:φₖ₊₁ = wrap(φₖ + ωₖΔt),   vₖ₊₁ = vₖ,   ωₖ₊₁ = ωₖ|(3.4)]]

When |ωₖ| is below the configured epsilon, Equations (3.2)-(3.3) use their straight-line limits pₓ,ₖ₊₁ = pₓ,ₖ + vₖ cos(φₖ)Δt and pᵧ,ₖ₊₁ = pᵧ,ₖ + vₖ sin(φₖ)Δt. The nonlinear map f(·) is linearized numerically about the current estimate to obtain Fₖ. The prediction step is therefore

[[EQUATION:x̂ₖ⁻ = f(x̂ₖ₋₁⁺, Δt),   Pₖ⁻ = FₖPₖ₋₁⁺Fₖᵀ + Qₖ|(3.5)]]

The LiDAR cluster centre supplies a position-only measurement zₖ = [pₓ,meas, pᵧ,meas]ᵀ with H = [I₂×₂  0₂×₃]. The innovation, innovation covariance, Kalman gain, and corrected state are

[[EQUATION:yₖ = zₖ − Hx̂ₖ⁻,   Sₖ = HPₖ⁻Hᵀ + Rₖ,   Kₖ = Pₖ⁻HᵀSₖ⁻¹|(3.6)]]

[[EQUATION:x̂ₖ⁺ = x̂ₖ⁻ + Kₖyₖ|(3.7)]]

The code updates covariance in Joseph form,

[[EQUATION:Pₖ⁺ = (I − KₖH)Pₖ⁻(I − KₖH)ᵀ + KₖRₖKₖᵀ|(3.8)]]

which preserves symmetry and is numerically safer than the abbreviated covariance update. This prediction-correction cycle converts discontinuous LiDAR detections into continuous target tracks, a role consistent with maritime tracking literature in which filtering compensates for missed or noisy detections (Xie et al., 2024).

The implementation also includes a notion of stable motion before certain dynamic behaviours become trusted. This is a sensible safeguard. A target with only one or two noisy observations should not immediately be allowed to determine encounter type. The trade-off, however, is delayed response. The system becomes less vulnerable to false dynamic classification, but it can initially treat a real moving vessel as if it were static until the track stabilizes.

## 3.6 CPA metrics and virtual obstacles

For an obstacle at relative position `r` and relative velocity `v`, the standard closest-point-of-approach quantities are the predicted time to CPA and the distance at CPA. The code calls a dedicated `apf_cpa_metrics` helper while selecting the primary COLREG strategy, and it records `tcpa_s` and `dcpa_m` alongside the chosen controller state. This is more than an internal diagnostic. It means encounter selection is explicitly ordered by both rule priority and predicted risk.

In the current implementation, the selected target for rule reasoning is not simply the nearest obstacle. The controller evaluates obstacles, computes CPA-related scores, and ranks them through a combined score tuple that incorporates rule priority, TCPA, and DCPA. This ranking is an important design feature because it recognizes that the most operationally relevant target is not always the nearest one. A farther target on a converging course may deserve earlier attention than a nearer target that is already diverging or safely offset.

Virtual obstacles extend this idea into the control layer. When prediction is enabled, the controller can synthesize future obstacle instances corresponding to predicted collision-relevant positions. These are then handled by the APF similarly to ordinary obstacles. The engineering logic is straightforward: if the future path is dangerous, act now rather than waiting for the real obstacle to enter the immediate repulsive region. This is how the present system partially compensates for the short-horizon nature of APF without implementing a full receding-horizon optimizer.

## 3.7 Encounter classification and rule routing

The most explicitly COLREG-oriented part of the code is the encounter routing logic. The unified controller maintains sets of rule categories for crossing, overtaking, and head-on behaviour and dispatches APF control accordingly. The selection logic uses relative obstacle position, relative motion, and CPA-derived context. Once an encounter type is identified, the controller marks a selected controller profile and, where necessary, an active encounter profile name.

This mechanism matters for two reasons. First, it separates behavioural semantics from field mechanics. Instead of one monolithic APF trying to do everything, the system chooses between profile families appropriate to different encounter types. Second, it preserves interpretability. The selected controller is written into snapshots and can be traced alongside the associated encounter mode, CPA values, and APF state.

The routing design reflects a pragmatic COLREG translation. Head-on and overtaking are grouped through the overtaking/head-on controller branch, while crossing uses a separate controller branch imported from the crossing module. Default behaviour falls back to a more generic APF response. This is not a legal reasoner. It is a heuristic classifier plus routed local control. But it is still a major step beyond purely unstructured repulsion because it makes the intended response class explicit.

Algorithm 1 summarizes the executable routing path reconstructed from `select_colreg_strategy()` and `compute_apf_control()` in `src/laptop.py`. It preserves the code’s actual ordering: observed and virtual obstacles are screened, classified, scored by rule priority and CPA context, and then dispatched to the matching APF branch. The pseudocode deliberately omits logging and exception-handling details so that the decision logic remains visible.

Figure 2 presents the same pseudocode as an engineering flowchart, separating candidate evaluation and risk ranking from the final controller-dispatch stage.

[[FIGURE:F:/Desk/IP_test5/paper/assets/colreg_apf_dispatch_flowchart.png|Figure 2. Flowchart of COLREG encounter selection, risk ranking, and APF controller dispatch reconstructed from src/laptop.py.]]

## 3.8 Size-aware APF design

The unified APF parameter set reveals how obstacle size is operationalized. The controller stores scaling coefficients such as `apf_avoidance_pc_scale` and `apf_direction_pc_scale`, as well as activation angles, route lookahead distance, prediction horizon, dynamic speed thresholds, and side-lock timing. These parameters indicate that the APF is not merely a point obstacle repulsion with a goal attraction term. It is an encounter-aware local planner whose effective obstacle influence depends on geometry, relative motion, and the need to preserve chosen passing-side behaviour.

In the canonical APF interpretation, navigation is expressed by a scalar potential whose negative gradient produces the commanded direction (Khatib, 1986). A useful decomposition for this controller is

[[EQUATION:U(q) = U_att(q) + ΣⱼU_rep,ⱼ(q) + U_rule(q),   F(q) = −∇U(q)|(3.9)]]

For a goal or look-ahead point q_g, the classical attractive potential is U_att(q) = ½k_att‖q − q_g‖². The implementation uses the corresponding saturated force so that distant goals do not dominate obstacle avoidance:

[[EQUATION:F_goal = k_g min(d_g, d_sat)(q_g − q)/d_g,   d_g = ‖q_g − q‖|(3.10)]]

For comparison, the classical circular APF branch uses the repulsive-gradient weight (1/ρ − 1/ρ₀)/ρ² inside the influence distance ρ₀ and zero outside it. This is the standard finite-influence repulsion associated with Khatib's formulation (Khatib, 1986). The project-specific branch replaces circular distance with an ellipse level derived from the observed principal dimensions:

[[EQUATION:λⱼ(q) = {[(e_l,ⱼᵀ(q − oⱼ))/aⱼ]² + [(e_w,ⱼᵀ(q − oⱼ))/bⱼ]²}¹ᐟ²|(3.11)]]

[[EQUATION:aⱼ = ½s_pc PC1ⱼ + r_own,   bⱼ = ½s_pc PC2ⱼ + r_own|(3.12)]]

Here oⱼ is the obstacle centre; e_l,ⱼ and e_w,ⱼ are its principal length and width axes; s_pc is the configured scale; and r_own is the own-vessel equivalent radius. Encounter-specific longitudinal and lateral multipliers are applied in the code but omitted from Equation (3.12) for clarity. The cluster-size branch then uses

[[EQUATION:wⱼ = clip(1 − λⱼ, 0, 1),   F_rep,ⱼ = k_rep,ⱼ wⱼ n̂ⱼ|(3.13)]]

where n̂ⱼ is the normalized outward gradient of the ellipse. Observed obstacles and EKF-predicted virtual obstacles contribute separate repulsive terms. The controller adds path attraction and COLREG/side-lock steering before converting the resultant into a bounded heading command:

[[EQUATION:F_res = F_goal + F_path + ΣⱼF_rep,ⱼ + F_rule,   ψ_d = clip(atan2(F_res,y, F_res,x), ±ψ_max)|(3.14)]]

This formulation remains local and can inherit APF local-minimum or oscillation problems, but size-dependent ellipses, predicted virtual obstacles, and rule-biased lateral forces make the field more suitable for maritime encounters than generic point-obstacle repulsion (Lyu et al., 2024; Li et al., 2024a).

When cluster-size scaling is enabled, the effective obstacle field is built from PC-based geometry. In practical terms, the target is treated through an elliptical field whose half-axes depend on observed target size plus an own-vessel equivalent radius. This gives the own vessel more lateral or longitudinal clearance depending on target extent. When scaling is disabled, the controller reverts to a classic fixed influence distance.

This switch is crucial for interpretation. If trajectories differ meaningfully between large and small targets under size-aware APF but not under classic APF, then the size signal is functioning as intended. The dissertation later shows that this is broadly true in three scenario pairs, though not uniformly in the head-on case.

Figure 3 shows how these terms are generated and combined in the current controller. The left panel visualizes an attractive basin around the look-ahead goal together with the repulsive peaks created by a PC1/PC2 obstacle ellipse and an EKF-predicted virtual obstacle. The right panel maps the same construction to the implemented vector decomposition. The figure is conceptual rather than a snapshot of a particular experiment; its geometry and force terms are reconstructed from `ellipse_level_and_away()`, `apf_goal_attraction_body()`, `apf_repulsion_for_obstacle()`, and `compute_apf_control()` in `src/laptop-crossing.py`.

[[FIGURE:F:/Desk/IP_test5/paper/assets/apf_potential_field_generation.png|Figure 3. Author-generated schematic of the size-aware predictive artificial potential field and code-aligned force decomposition; APF principles follow Khatib (1986) and Lyu et al. (2024).]]

## 3.9 Side locking and passage preference

Purely reactive fields often suffer from oscillation: the controller starts to pass one side of an obstacle, then a small geometry change causes the lateral component to reverse, leading to indecisive motion. The code addresses this by including a side-lock duration and an exit-level condition. This is a subtle but important control feature. Maritime rule compliance is not only about choosing the right side; it is also about remaining committed enough that the manoeuvre becomes legible.

The controller also contains parameters for stern-direction, bow-direction, and endpoint direction helpers. These reveal that the APF has explicit support for passage preference, especially in crossing and overtaking contexts where “pass ahead” and “pass astern” are not symmetric choices. The field therefore encodes more than obstacle avoidance. It encodes a preferred route geometry relative to the target.

## 3.10 Control output generation

At the final stage, the APF resultant is converted into heading-oriented action. The control command is not a full six-degree-of-freedom vessel optimization. Instead, the controller computes a direction, limits heading step size, and enforces operating-speed conventions. This has two consequences.

The first is computational simplicity. The controller can update rapidly and react online without solving a heavy optimization problem.

The second is that the low-level manoeuvre space is constrained. The vehicle essentially expresses avoidance through heading change and bounded forward motion rather than through a broad set of trajectory primitives. This helps explain why overtaking remains difficult. Near-collinear interactions are precisely the cases where small heading-only corrections may struggle to create sufficient separation while preserving smooth progression.

## 3.11 Logging, snapshots, and interpretability

One of the strongest engineering features of the codebase is its logging structure. The controller writes CSV context fields and obstacle snapshots that include environment name, switch state, selected controller, active profile, and APF-related decision data. This makes it possible to reconstruct not only where the vessel went, but why the controller believed it was acting in a certain way.

That audit trail is essential for this dissertation’s methodology. Without it, the supplied figures would be merely qualitative trajectory plots. With it, they become evidence of a deeper decision process. A useful autonomous maritime controller should be explainable after the fact. The present system approaches that ideal by exposing exactly which features of the perception and rule layers were active at each stage of a run.

## 3.12 Architectural summary

In summary, the implemented system can be characterized as an interpretable local collision-avoidance architecture with predictive augmentation. It does not implement the full scoping vision of Behaviour Tree orchestration and GPMP2 planning. It does, however, implement a meaningful intermediate architecture:

- LiDAR clusters become geometric targets.
- Geometric targets become tracked dynamic entities.
- Dynamic entities become encounter categories and predicted risks.
- Encounter categories route the controller into size-aware APF variants.
- APF variants generate bounded heading-oriented action.

The rest of this dissertation evaluates how well this architecture behaves under the specific Webots encounter scenarios available in the supplied experiments.

\pagebreak

# 4 Experimental Design

## 4.1 Simulation environment

The experimental platform for this dissertation is Webots, a mature robotics simulation environment widely used for repeatable controller development and sensor integration studies (Cyberbotics, 2026). The current project uses Webots not as a visually rich demonstration platform but as a controllable environment in which obstacle geometry, own-vessel motion, and encounter types can be compared across switch settings. This distinction matters. The purpose of the simulation campaign is not photorealism. It is controlled comparison.

The supplied figure sets show four encounter families:

- crossing from left to right;
- crossing from right to left;
- head-on approach;
- overtaking.

Each family is represented twice, once with a small target vessel and once with a large target vessel. This yields the crucial experimental pairing required to test size-aware behaviour. For each environment-size pair, a four-switch comparison figure is also available, capturing runs with EKF and cluster-size APF turned on or off.

The simulated motion is scaled and simplified relative to real maritime operations. This is normal for an MSc-level controller study, but it must be stated explicitly because the distances and clearances in the figures are not directly transferable to full-scale ships. The operational meaning of a clearance margin in this work is therefore primarily comparative within the simulation, not a direct claim about full-scale safe passing distance.

## 4.2 Ablation structure

The controller is evaluated under a `2 x 2` ablation design:

- EKF prediction enabled or disabled;
- cluster-size APF scaling enabled or disabled.

This yields four switch combinations for each of the four encounter families and each of the two target sizes, resulting in 32 selected runs for the supplied four-switch trajectory comparisons. In addition, the supplied pairwise trajectory plots compare the `EKF on + cluster on` configuration for large and small target vessels within each encounter family. The principal-component time-series plots likewise compare large and small targets under the same paired configuration.

This structure supports three distinct analyses.

The first is within-scenario ablation: how much does EKF or size-awareness matter for one fixed encounter and target size?

The second is between-size comparison: does a large target produce a measurably different obstacle-size signature and a different avoidance path than a small target in the same scenario?

The third is cross-scenario comparison: do certain encounter families remain inherently harder than others under the present controller design?

The experiments do not support strong population-level inference because each cell in the matrix is represented by a selected run rather than a repeated randomized batch. However, they do support structured mechanism analysis, which is the standard of evidence adopted in this dissertation.

## 4.3 Metrics

The most important reported metrics in this work are not all scalar success/failure indicators. Three classes of evidence are used.

First, trajectory geometry. The path of the own vessel shows where avoidance begins, how far the vessel deviates laterally, whether it changes side mid-manoeuvre, and whether it returns smoothly toward its nominal route. For a rule-aware controller, these geometric characteristics matter because they are part of manoeuvre legibility.

Second, obstacle-size evolution. The PC1 and PC2 time series show whether the perception layer meaningfully distinguishes the large and small targets. They also show whether size estimation is stable through time or corrupted by viewpoint and association effects.

Third, minimum clearance. The post-processing script computes clearance as centre-to-centre distance minus an interpolated collision boundary based on equivalent geometric extents. This is a useful proxy for comparative safety margin, but it is still a proxy. A negative value indicates overlap of equivalent boundaries, not necessarily full mesh contact in the simulator.

Together these metrics provide a stronger picture than any single scalar would. A run with positive clearance but unstable oscillation or late reversal may still be undesirable. A run with a slightly negative equivalent clearance but otherwise smooth, early, and interpretable behaviour may still reveal useful control structure, especially when compared across switch settings.

## 4.4 Success-threshold inconsistency

A major methodological issue discovered during analysis is that the already-generated figure labels do not all use the same success criterion. Some crossing figures treat slightly negative minimum clearance as failure. Some head-on and overtaking figures explicitly state a success threshold of clearance greater than `-0.30 m`. The current post-processing code reports minimum clearance values but the supplied plots reflect at least two threshold conventions across scenario families.

This inconsistency has a direct consequence: global success-rate claims across all scenarios would be methodologically weak and potentially misleading. The same physical trajectory could be classified differently depending on which plotting rule was used. For this reason, the dissertation treats minimum clearance and trajectory behaviour as the primary reported evidence, and scenario-level success labels as secondary annotations that must be interpreted within their own plotting context.

This decision is not an attempt to rescue bad results. It is the opposite. It is an attempt to ensure that the results chapter reports only those inferences that the current analysis pipeline can actually support.

## 4.5 Data provenance

The supplied figures identify specific run timestamps for the paired large-versus-small comparisons. This is useful because it ties visual outputs back to logged runs rather than presenting anonymous plots. The project’s logging pipeline stores obstacle snapshots and controller context, which means the figures can be understood as derived artefacts from a traceable run history.

The selected paired runs reported in the supplied figures are:

- left-to-right crossing: large `run_20260625_185755`, small `run_20260625_163234`;
- right-to-left crossing: large `run_20260625_162046`, small `run_20260625_160520`;
- head-on: large `run_20260625_191229`, small `run_20260625_191930`;
- overtaking: large `run_20260625_192948`, small `run_20260625_193234`.

The existence of these explicit identifiers improves reproducibility in one sense, but not in another. It allows the user to track which logged run is being shown, yet it does not substitute for repeated randomized testing. These are still selected runs.

## 4.6 Threats to validity

There are several threats to validity, and they should be made explicit before the results are interpreted.

The first threat is single-run representation. With one selected run per matrix cell, it is impossible to estimate variance or statistical robustness.

The second threat is scenario-classification drift. Some internal logs and downstream labels indicate that the online encounter classifier does not always align perfectly with the nominal world-file scenario name.

The third threat is perception uncertainty. LiDAR cluster geometry may fluctuate because of viewpoint, occlusion, or partial observation, particularly when the target’s apparent extent changes rapidly.

The fourth threat is proxy safety measurement. Equivalent clearance is informative, but it is not identical to true hull-to-hull separation.

The fifth threat is simulation scaling. The numerical magnitudes in Webots are valuable for comparative analysis inside the simulation, but they should not be interpreted naively as full-scale operational thresholds.

The sixth threat is implementation inheritance. Because the unified controller reuses and synchronizes functions from earlier scenario-specific modules, some behavioural choices reflect the historical evolution of the codebase rather than a clean global re-design.

These threats do not invalidate the dissertation. They define the correct scope of its claims.

## 4.7 Why the current protocol is still useful

Despite those limitations, the protocol remains useful for three reasons.

First, the ablation is structurally clean. Turning EKF or size-awareness on and off isolates meaningful architectural factors.

Second, the supplied figures are sufficiently rich to expose concrete mechanism behaviour. They show not only final success or failure labels but also path shape, side choice, and obstacle-size evolution.

Third, the protocol is honest enough to support critical engineering interpretation. It reveals failure modes rather than hiding them, especially in overtaking.

For an MSc dissertation grounded in an existing codebase, this is a legitimate and valuable level of empirical analysis. The point is not to claim finality; it is to produce a rigorous account of what the present system actually does.

\pagebreak

# 5 Results

## 5.1 Large-versus-small target trajectory comparisons

The first result set compares avoidance trajectories for large and small targets in each of the four encounter families under the `EKF on + cluster on` configuration. These plots provide the clearest visual indication of whether size-aware obstacle modelling materially changes motion.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/colreg_size_trajectories/mr_webots_cross_left_to_right_small_ship__mr_webots_cross_left_to_right_large_ship.png|Figure 4. Left-to-right crossing trajectory comparison for large and small target vessels under EKF-on and size-aware APF-on conditions.]]

In the left-to-right crossing case, both trajectories begin from a nearly common northbound path and then deviate eastward to avoid the crossing target. The large-target case shows a visibly larger lateral excursion than the small-target case. This is the expected qualitative behaviour if PC-based obstacle scaling is functioning correctly: a larger observed target should induce an earlier or stronger lateral repulsion and consequently a broader detour. The legend values support that interpretation by reporting a larger PC1 for the large vessel than for the small vessel. Equally important, both trajectories remain smooth and do not exhibit a visible side reversal. This suggests that in this scenario the controller can use size information without sacrificing path coherence.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/colreg_size_trajectories/mr_webots_cross_right_to_left_small_ship__mr_webots_cross_right_to_left_large_ship.png|Figure 5. Right-to-left crossing trajectory comparison for large and small target vessels under EKF-on and size-aware APF-on conditions.]]

The right-to-left crossing case is more subtle. Both trajectories still separate, but the spatial relationship between them differs from the previous case. The small-target trajectory bends further east than the large-target trajectory through the middle of the encounter. This does not automatically mean the size logic failed, because encounter geometry and route bias can interact with obstacle shape in ways that produce different lateral timing. However, it immediately signals that interpretation should not depend on trajectory amplitude alone. The obstacle-size signal must be examined together with the time-series data and the four-switch comparisons.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/colreg_size_trajectories/mr_webots_head_on_small_ship__mr_webots_head_on_large_ship.png|Figure 6. Head-on trajectory comparison for large and small target vessels under EKF-on and size-aware APF-on conditions.]]

The head-on case yields one of the most informative anomalies in the whole dataset. Both trajectories perform a clear side-stepping manoeuvre, but the small-target path swings farther east than the large-target path for much of the encounter. At face value this looks inverted: one might expect the larger target to induce the stronger response. The legend values show that the large-vessel PC1 is actually reported as slightly smaller than the small-vessel PC1 in this scenario. That reversal already hints that the issue originates upstream, in perception and geometry estimation, rather than purely in control. The controller is acting on the geometry it was given. If that geometry is reversed, the manoeuvre may also be reversed.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/colreg_size_trajectories/mr_webots_overtaking_small_ship__mr_webots_overtaking_large_ship.png|Figure 7. Overtaking trajectory comparison for large and small target vessels under EKF-on and size-aware APF-on conditions.]]

The overtaking case differs markedly from the others. The two trajectories are much closer for a large part of the run, and the main separation appears in the mid-course before both tracks reconverge. This is consistent with the later four-switch evidence that overtaking remains the weakest encounter family under the current controller. The size difference is visible, but it does not rescue the scenario. This is an important distinction. A perception feature can be functioning in a local sense without being sufficient to solve the dominant control problem in that encounter class.

Across the four paired trajectory plots, a general result emerges. Size-aware behaviour is not merely decorative: large and small targets often produce meaningfully different path shapes. But the sign and value of those differences remain scenario dependent, which means the interpretation must be anchored in the PC time-series rather than in path amplitude alone.

## 5.2 Principal-component time-series comparisons

The second result set examines PC1 and PC2 over time for the same paired large-versus-small comparisons. These plots are central because they test whether the controller’s size-aware APF is being fed a stable and meaningful geometry signal.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/pc_size_comparisons/mr_webots_cross_left_to_right_small_ship__mr_webots_cross_left_to_right_large_ship_pc1_pc2.png|Figure 8. Principal-component dimensions over time for the left-to-right crossing case, comparing large and small target vessels.]]

The left-to-right crossing comparison shows the clearest success case for size discrimination. The large-vessel PC1 remains substantially above the small-vessel PC1 for most of the encounter, while PC2 remains relatively similar and much smaller than PC1 in both cases. This is consistent with an elongated target silhouette whose dominant observed extent is substantially larger for the large vessel. The corresponding trajectory plot aligns with this result: the larger obstacle estimate produces the larger deflection.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/pc_size_comparisons/mr_webots_cross_right_to_left_small_ship__mr_webots_cross_right_to_left_large_ship_pc1_pc2.png|Figure 9. Principal-component dimensions over time for the right-to-left crossing case, comparing large and small target vessels.]]

The right-to-left crossing case also exhibits strong size separation, though the PC1 signal for the large vessel includes a dramatic mid-encounter drop and later recovery. This suggests one of two things: either the apparent geometry of the target changed significantly with viewpoint, or the cluster association temporarily altered the observed principal axis. In either case, the geometry signal is usable but not perfectly stable. The controller’s behaviour under such fluctuation becomes a measure of robustness rather than of perception correctness alone.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/pc_size_comparisons/mr_webots_head_on_small_ship__mr_webots_head_on_large_ship_pc1_pc2.png|Figure 10. Principal-component dimensions over time for the head-on case, comparing large and small target vessels.]]

The head-on case confirms the anomaly noted earlier. The reported large-vessel PC1 is not consistently larger than the small-vessel PC1. In fact, the paired figure labels indicate a median PC1 ordering of roughly `0.59 m` for the large vessel and `0.65 m` for the small vessel. This inversion undermines any naive claim that size-aware APF “always” distinguishes target scale correctly. What it actually shows is more nuanced: the geometry estimator often captures target scale, but in head-on observations it may be distorted by partial frontal visibility and by the difficulty of maintaining consistent principal-axis interpretation under changing perspective.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/pc_size_comparisons/mr_webots_overtaking_small_ship__mr_webots_overtaking_large_ship_pc1_pc2.png|Figure 11. Principal-component dimensions over time for the overtaking case, comparing large and small target vessels.]]

The overtaking case again shows useful but incomplete size separation. The large-vessel PC1 is substantially greater than the small-vessel PC1 for much of the encounter, yet this does not translate into successful overtaking behaviour in the four-switch results. The implication is important: perception quality is a necessary but not sufficient condition for good control. Even when the APF knows the target is larger, the overtaking strategy can still fail because the relative-motion geometry itself remains difficult.

Taken together, the PC plots justify one of the dissertation’s central conclusions. Cluster-based geometry scaling is meaningful and usually informative, but it is not universally reliable. The head-on anomaly makes it clear that the controller’s size-awareness is only as trustworthy as the upstream cluster geometry and track association.

## 5.3 Four-switch trajectory results: left-to-right crossing

The left-to-right crossing four-switch comparison is the strongest all-round result in the dataset.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/mr_webots_cross_left_to_right_large_ship_four_switch_robot_trajectories.png|Figure 12. Four-switch trajectory comparison for the left-to-right crossing case with a large target vessel.]]

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/mr_webots_cross_left_to_right_small_ship_four_switch_robot_trajectories.png|Figure 13. Four-switch trajectory comparison for the left-to-right crossing case with a small target vessel.]]

For the large target, three of the four switch combinations are labelled as successful in the supplied figure, with only the `EKF on, cluster APF off` case failing under the figure’s original threshold. For the small target, all four switch combinations are labelled successful. The specific interpretation should be cautious because of the threshold inconsistency discussed earlier, but the broad pattern is still useful. Left-to-right crossing appears to be the encounter class in which the controller is most robust even when prediction or size-awareness is removed.

This suggests two things. First, the geometry of this encounter may already align well with the own vessel’s route and APF bias structure, making it easier for the controller to produce a coherent detour. Second, it implies that not all encounter families equally stress the architecture. This matters because a controller can look more mature than it is if judged only on an easier crossing configuration.

An interesting detail is that the `EKF off, cluster APF on` case remains successful and even yields a comparatively large clearance in the plotted annotation. This indicates that when encounter geometry is favourable, size-aware geometric repulsion alone may be enough to produce safe local behaviour. It does not reduce the importance of EKF globally, but it shows that prediction benefits are scenario dependent rather than uniform.

## 5.4 Four-switch trajectory results: right-to-left crossing

The right-to-left crossing family is much more discriminating.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/mr_webots_cross_right_to_left_large_ship_four_switch_robot_trajectories.png|Figure 14. Four-switch trajectory comparison for the right-to-left crossing case with a large target vessel.]]

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/mr_webots_cross_right_to_left_small_ship_four_switch_robot_trajectories.png|Figure 15. Four-switch trajectory comparison for the right-to-left crossing case with a small target vessel.]]

In both large and small target variants, only the `EKF on, cluster APF on` case is labelled successful in the supplied plots. All three alternatives fail. This is the clearest evidence in the dataset that EKF prediction and size-aware APF together provide a compound benefit. It is not just that prediction helps a little, or that size-awareness is cosmetically different. Rather, without both, the controller becomes much more vulnerable to near-conflict trajectories in this encounter geometry.

The minimum clearances reported in the figure annotations reinforce that reading. The successful runs remain only slightly positive, which means the controller is operating near the boundary of acceptable behaviour even in its best configuration. The failed runs show negative clearances, with the worst cases substantially inside the equivalent boundary threshold. This is a strong argument against over-generalizing the left-to-right crossing success. The architecture is not uniformly robust across all crossing directions.

A likely explanation is that the preferred passing side, route-following bias, and own-target geometry interact differently when the target comes from the opposite lateral direction. The right-to-left case may demand a clearer rule-based commitment or earlier projection of future conflict than the simpler APF geometry can provide unless prediction and size structure are both active.

## 5.5 Four-switch trajectory results: head-on

The head-on family occupies an intermediate position between partial success and persistent fragility.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/mr_webots_head_on_large_ship_four_switch_robot_trajectories.png|Figure 16. Four-switch trajectory comparison for the head-on case with a large target vessel.]]

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/mr_webots_head_on_small_ship_four_switch_robot_trajectories.png|Figure 17. Four-switch trajectory comparison for the head-on case with a small target vessel.]]

Each of the head-on plots reports two successful and two failed runs under a success threshold of clearance greater than `-0.30 m`. The success labels therefore must be interpreted as tolerance-based rather than strict non-overlap classifications. Even with that caveat, the scenario remains informative. The `EKF on` cases are consistently better than the `EKF off` cases in both large and small target versions. This supports the general claim that motion prediction matters strongly whenever dynamic relative approach is central to the encounter.

The head-on case also illustrates a nuanced point about the interaction between perception and control. Although the PC1 estimates for large and small targets are partially inverted in the paired size plots, the controller still produces recognizably different manoeuvres across switch settings. This suggests that encounter-specific routing and prediction contribute more to head-on safety than size-awareness alone. In other words, head-on behaviour under the present architecture is prediction dominated rather than geometry dominated.

The failed EKF-off runs also underscore why purely reactive APF is inadequate in dynamic encounters. A head-on target is a case where early prediction of closure is crucial; waiting until local geometry alone becomes dangerous leaves too little room for a clear, smooth alteration.

## 5.6 Four-switch trajectory results: overtaking

The overtaking family is the clearest failure class in the entire dataset.

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/mr_webots_overtaking_large_ship_four_switch_robot_trajectories.png|Figure 18. Four-switch trajectory comparison for the overtaking case with a large target vessel.]]

[[FIGURE:F:/Desk/IP_test5/logs/generated_figures/mr_webots_overtaking_small_ship_four_switch_robot_trajectories.png|Figure 19. Four-switch trajectory comparison for the overtaking case with a small target vessel.]]

All four switch combinations are labelled failed for both the large and small target variants. This is true even under the tolerance-based success convention used in the head-on and overtaking plots. The negative clearances vary in magnitude, but no combination produces a convincingly successful overtaking pass.

This result is arguably the most important empirical finding of the dissertation because it defines the main boundary of the current architecture. Overtaking is not just “a bit worse” than other encounter families. It is structurally unresolved. The presence of meaningful PC1 separation between large and small targets in the paired size plots shows that perception is not completely blind. The existence of EKF prediction also does not rescue the scenario. Therefore, the dominant weakness lies in the encounter-specific control logic itself.

Overtaking is difficult because the own vessel and target vessel remain near-collinear for long periods, making lateral passage choice and completion criteria more delicate than in crossing. A local APF with heading-limited control may struggle to create enough lateral displacement early enough while still preserving forward progress. Furthermore, if the classifier or field logic oscillates between “follow behind” and “pass around” interpretations, the manoeuvre can remain trapped in an unsafe near-track geometry. The overtaking results strongly suggest that a dedicated overtaking design, rather than a lightly adapted APF branch, is required.

## 5.7 Consolidated interpretation of results

Three high-level findings are supported by the results.

The first finding is that EKF prediction is the strongest enabling component for dynamic encounters. This is most obvious in right-to-left crossing and head-on cases, where EKF-on runs outperform EKF-off alternatives.

The second finding is that cluster-size-based APF scaling is useful but not uniformly decisive. It meaningfully differentiates large and small targets in most paired comparisons, but its effect on safety outcome is secondary to the broader encounter geometry and prediction quality. Its biggest weakness is the head-on size inversion, which indicates fragility in the upstream geometry pipeline.

The third finding is that overtaking remains unresolved. This single result is enough to reject any blanket claim of full COLREG-compliant autonomy under the present implementation.

The results therefore support a careful conclusion: the architecture is promising as an interpretable, modular, prediction-augmented APF controller, but it is not yet mature enough to justify stronger compliance claims.

\pagebreak

# 6 Discussion

## 6.1 Why EKF prediction mattered most

The most consistent pattern across the supplied experiments is the advantage of EKF-based prediction in dynamic encounter handling. This finding is unsurprising in theory but important in practice because it demonstrates that the controller’s most valuable improvement did not come from more complicated local field shaping alone. It came from better temporal understanding of the target.

Without EKF prediction, the controller loses more than a velocity estimate. It loses its ability to reason meaningfully about closure, to prioritize targets by future risk rather than current distance alone, and to project conflicts into virtual obstacles. In effect, a dynamic target becomes a sequence of local geometric observations. That degradation is especially damaging in head-on and right-to-left crossing cases because these scenarios demand action before the obstacle is physically close enough for pure repulsion to be reliable. The results align with the broader literature in this respect. Whether one uses VO, MPC, or simpler predictive heuristics, maritime avoidance becomes much more effective when future convergence is modelled rather than inferred late from proximity (Cho et al., 2019; Thyri and Breivik, 2022; Zhang et al., 2022).

The current implementation reveals an additional engineering insight: prediction improves not only safety margin but decision continuity. Once a stable track exists, the controller can retain a consistent interpretation of the obstacle through time. That helps preserve encounter mode, selected controller profile, and APF side-lock behaviour. A purely reactive geometric controller can jitter because the meaning of each obstacle is reconstructed from scratch at each instant. Tracking reduces that fragility.

## 6.2 What size-aware APF actually contributed

The value of cluster-size-based APF scaling must be interpreted carefully. It would be misleading to say that size-awareness “solved” any encounter class. The overtaking failures alone disprove such a statement. It would also be incomplete to say that size-awareness did not matter. In the paired trajectory and PC plots, large and small targets often produced clearly different obstacle-size histories and different path shapes. That means the size signal is entering the control process in an operationally meaningful way.

The correct interpretation is that size-awareness improves the physical realism of local risk representation. A larger target should, all else equal, create a wider effective obstacle field and a broader passing response. The left-to-right and right-to-left crossing results broadly support this. The head-on anomaly does not negate the concept; it exposes the fragility of its measurement channel. In other words, the design idea is sound but dependent on the stability of LiDAR-derived geometry.

This also highlights an important distinction between perception-driven adaptation and guarantee. The controller can use size information adaptively without thereby guaranteeing better safety outcome in every case. If another part of the control logic is weak, as in overtaking, then better geometry alone does not rescue the encounter. The contribution of size-awareness is therefore conditional: it sharpens the local field, but it does not replace the need for strong encounter-specific strategy.

## 6.3 The head-on size inversion as a perception diagnostic

The head-on reversal of large and small PC1 medians is not just a statistical curiosity. It is a diagnostic event. It shows that the obstacle-size pipeline is vulnerable to encounter geometry. When a vessel is viewed more frontally, the visible extent along its major axis may collapse relative to side-on observations. In a point-cloud setting, the principal-axis estimate can also be distorted when only a limited frontal portion of the target is observed or when the cluster rotates in apparent orientation across frames.

This has two implications. First, it means that PC1 and PC2 should be interpreted as observed extents rather than ground-truth vessel dimensions. Second, it suggests that future versions of the controller should propagate uncertainty in size estimation rather than using only point estimates. For example, a track could maintain not just average PC1 and PC2 values but also confidence intervals or conservative upper quantiles. The APF could then enlarge safety margins when geometry estimates are unstable. This would be a natural next step and is consistent with broader sensor-fusion and uncertainty-aware tracking literature (von Brandis et al., 2024; Wu et al., 2026).

## 6.4 Why overtaking failed

The overtaking failure deserves detailed interpretation because it sets the clearest boundary of the current work.

In overtaking, the own vessel approaches a slower target from behind with near-aligned course. This geometry is fundamentally harder for a local heading-bias controller than crossing or head-on encounters. The own vessel must decide early on which side to pass, create enough lateral offset to clear the target, maintain that offset while continuing forward, and then decide when it is safe to rejoin the original route. Each of these steps is difficult under the current architecture.

First, the relative-motion signal is weak in the lateral direction because both vessels travel in nearly the same longitudinal direction. A controller that depends strongly on local lateral repulsion may not receive a sufficiently strong or stable cue to initiate decisive passing.

Second, overtaking demands a completion condition. It is not enough to deviate around the target; the own vessel must know when the pass is complete and when it can safely recover. The current implementation does not appear to contain a fully explicit overtaking completion logic comparable to what one might build around target stern and bow reference points.

Third, heading-limited APF control may be too weak as a manoeuvre generator for near-collinear passage. Unlike a crossing case, where a strong lateral bias naturally creates separation, overtaking may require more deliberate multi-stage behaviour: move laterally, maintain offset, advance until clear of the bow, and only then merge back.

Fourth, any instability in encounter classification or side locking becomes especially damaging because the geometry offers fewer natural cues for recovery. In a crossing situation, wrong-side motion may still generate visible separation. In overtaking, small logic errors can keep the own vessel trapped near the target’s wake line.

These points suggest that the overtaking problem is not merely a tuning issue. It is likely an architectural issue. A dedicated overtaking state machine or Behaviour Tree branch with explicit pass-initiation, pass-maintenance, and pass-completion states would be more appropriate than reliance on a lightly modified APF branch alone.

## 6.5 COLREG compliance versus rule-informed behaviour

One of the most important interpretive tasks in this dissertation is to resist a common overclaim in the literature: the assumption that scenario-specific geometric success is equivalent to COLREG compliance. The present results do not support that equation.

First, the success labels themselves are threshold sensitive and not globally harmonized across the already-generated plots. This alone prevents robust claim-making at the level of aggregate compliance rate.

Second, COLREG compliance requires more than avoiding overlap of equivalent geometric boundaries. It requires correct encounter identification, appropriate allocation of give-way or stand-on responsibility, sufficiently early and substantial action, and consistent manoeuvre legibility (IMO, 1972; Hu et al., 2022; Chang et al., 2024).

Third, parts of the implementation remain heuristic. The controller routes behaviour in a COLREG-inspired way, but it does not encode the full semantics of stand-on vessel monitoring or recovery logic under all conditions. It is therefore best described as rule-informed rather than proven rule-compliant.

This is not a weakness of the dissertation’s argument. It is one of its main contributions. By refusing to collapse all success into a single binary compliance label, the thesis preserves the difference between encouraging engineering evidence and stronger normative claims.

## 6.6 Relationship between the implemented system and the scoping vision

The current results also clarify the gap between the project’s original scoping vision and its implemented state. The scoping study was correct to view the navigation problem as requiring interpretable high-level decision logic plus stronger planning capability than pure reaction. The current codebase confirms that intuition indirectly. The most obvious failure modes of the present system are precisely those that a more explicit behavioural coordination layer and a stronger planning module might address.

For example, overtaking would benefit from a higher-level behavioural structure that commits to a passing stage sequence rather than relying on instantaneous field balance. Likewise, the persistence of threshold inconsistency and scenario-classification drift suggests that the system would benefit from more formalized state and validation structure, which a BT-based architecture could help organize. GPMP2 or MPC would then become plausible future additions once the controller’s encounter semantics are stable enough to justify longer-horizon trajectory optimization.

In this sense, the scoping study was not wrong. It was ahead of the implemented system. The dissertation’s task has been to connect the two honestly: to show that the current controller already solves a meaningful subset of the intended problem, while also showing why the originally scoped extensions remain relevant.

## 6.7 Threats to validity revisited

The results chapter already acknowledged the main threats to validity, but their interpretive consequences deserve emphasis.

Single-run evidence means that the dissertation cannot make statistical claims about robustness. What it can do is identify patterns strong enough to be visible across multiple scenario families, such as EKF importance and overtaking weakness.

Equivalent clearance as a metric is informative but limited. Negative clearances in the plots should not be paraphrased casually as “collisions” unless that correspondence has been separately verified at the mesh-contact level. They are better described as failures under the project’s boundary-proxy criterion.

Online classification drift means that some apparent scenario semantics may come from the nominal world setup rather than the controller’s own decision state. This reinforces the need to log and analyse controller-internal encounter labels explicitly.

Simulation scaling and limited disturbance injection mean that the results represent controller behaviour under controlled nominal conditions rather than a broad uncertainty envelope. Real-world clutter, sea state, and sensing degradation would likely enlarge the failure space.

These limitations are serious, but they do not erase the dissertation’s contributions. They define the exact level at which the evidence should be trusted.

## 6.8 Implications for future system development

The present findings suggest a concrete development order.

The first priority should be validation protocol repair. A project cannot meaningfully compare controller revisions if its success thresholds differ across scenario families. Harmonized post-processing and repeated-run evaluation are prerequisites for stronger future claims.

The second priority should be overtaking redesign. This is the dominant behavioural weakness and is unlikely to be solved by parameter tuning alone.

The third priority should be perception uncertainty handling. The head-on size inversion shows that geometry estimates need confidence-aware treatment.

The fourth priority should be higher-level behavioural formalization. Once the perception and validation layers are stronger, a formal Behaviour Tree becomes a practical next step rather than a speculative architecture note.

The fifth priority should be trajectory-planning augmentation. Only after the above layers are stable does it become worthwhile to add longer-horizon optimization such as GPMP2 or MPC for smoother, more globally coherent manoeuvres.

These recommendations follow directly from the experimental evidence. They are not generic future-work statements. They are a prioritized response to the actual current bottlenecks.

\pagebreak

# 7 Conclusions and Future Work

## 7.1 Conclusions

This dissertation set out to evaluate whether the implemented project code supports a meaningful COLREG-informed autonomous surface vessel collision-avoidance architecture in busy-waterway simulation. The answer is yes, but only within a clearly bounded sense.

The codebase implements a full perception-to-control chain combining LiDAR clustering, principal-component obstacle geometry, EKF target tracking, CPA-based risk prediction, encounter-aware routing, and a size-aware APF control layer. This chain is interpretable, loggable, and demonstrably operational in Webots across canonical encounter families.

The strongest empirical result is that EKF prediction materially improves dynamic encounter handling. The controller behaves much more robustly when it can estimate target motion and project future conflict than when it reacts only to instantaneous geometry.

The second major result is that cluster-based obstacle-size estimation is operationally meaningful. In most scenario pairs, the large and small targets produce differentiated PC1 histories and different avoidance trajectories. The head-on inversion shows that this signal is not universally reliable, but it does not nullify the broader usefulness of size-aware APF scaling.

The third and most constraining result is that overtaking remains unsolved under the current architecture. Both large and small overtaking scenario families fail under all four switch combinations shown in the supplied four-switch plots. This single result prevents any defensible claim of broad COLREG compliance.

The dissertation therefore concludes that the present system should be described as an interpretable, prediction-augmented, rule-informed local collision-avoidance architecture rather than as a fully verified COLREG-compliant navigation system. That conclusion is narrower than the most ambitious framing of the scoping study, but it is better aligned with the evidence and therefore academically stronger.

## 7.2 Main engineering lessons

Five engineering lessons emerge from the work.

First, prediction is not optional in dynamic encounters. Even a lightweight EKF can transform controller quality because it supports stable target identity, CPA reasoning, and proactive virtual-obstacle generation.

Second, obstacle shape matters. PC-based elliptical danger regions are more expressive than fixed circular influence distances and help adapt the controller to large and small targets.

Third, perception and control cannot be analysed separately. The head-on geometry inversion shows that controller behaviour may appear inconsistent even when the real issue originates in how the target was observed and represented.

Fourth, encounter-specific logic matters. Overtaking cannot be treated as a small variation of generic repulsion; it demands a dedicated strategy structure.

Fifth, validation discipline matters as much as controller design. Inconsistent success thresholds weaken interpretation even when the controller itself is technically interesting.

## 7.3 Future work

The immediate next steps are practical.

The first should be a harmonized validation pipeline with a single project-wide success definition, automatic consistency checks between plotting scripts and figure labels, and repeated randomized runs for each scenario-switch combination.

The second should be a redesign of overtaking logic around explicit stages: detection of overtaking geometry, side commitment, lateral offset build-up, safe bow-clear completion, and route recovery. Such logic is more naturally expressed in a state machine or Behaviour Tree than through APF shaping alone.

The third should be uncertainty-aware obstacle geometry. Tracks should store not only average PC1 and PC2 values but also stability indicators, allowing the APF to expand safety margins when size estimates are volatile.

The fourth should be improved multi-target reasoning. The current controller prioritizes one main target at a time, which is understandable for simplicity but restrictive in denser traffic.

The fifth should be formal top-layer behavioural integration. Behaviour Trees remain a sensible next architectural step because they can encode condition checking, fallback handling, and explicit overtaking stages while preserving interpretability.

The sixth should be longer-horizon planning augmentation. Once the behavioural layer is formalized and validated, GPMP2 or MPC can be added to improve path smoothness and to handle tighter constraints over a longer horizon.

The seventh should be broader sensing and real-platform validation. LiDAR-only geometry is sufficient for this dissertation’s scope, but AIS integration, IMU fusion, and eventually physical-platform trials would be needed before making stronger operational claims.

## 7.4 Final statement

The project has reached a meaningful intermediate milestone. It is no longer only a conceptual architecture described in a scoping document, yet it has not reached the level of mature, statistically validated maritime autonomy. The value of this dissertation lies in documenting that intermediate state with precision. It identifies what the implemented controller already demonstrates, where the supporting evidence is persuasive, and why the remaining limitations are technically specific rather than merely generic caveats. In that sense, the thesis contributes not merely a set of plots, but a defensible interpretation of an autonomous navigation prototype at its current stage of development.

\pagebreak

# References

[1] International Maritime Organization (1972). Convention on the International Regulations for Preventing Collisions at Sea (COLREGs). Available at: https://www.imo.org/en/about/conventions/pages/colreg.aspx

[2] Bae, I. and Hong, J. (2023). Survey on the Developments of Unmanned Marine Vehicles: Intelligence and Cooperation. Sensors, 23(10), 4643. https://doi.org/10.3390/s23104643

[3] Burmeister, H.-C. and Constapel, M. (2021). Autonomous Collision Avoidance at Sea: A Survey. Frontiers in Robotics and AI, 8, 739013. https://doi.org/10.3389/frobt.2021.739013

[4] Hu, L., Hu, H., Naeem, W. and Wang, Z. (2022). A review on COLREGs-compliant navigation of autonomous surface vehicles: From traditional to learning-based approaches. Journal of Automation and Intelligence, 1(1), 100003. https://doi.org/10.1016/j.jai.2022.100003

[5] Zhang, X., Wang, C., Jiang, L., An, L. and Yang, R. (2021). Collision-avoidance navigation systems for Maritime Autonomous Surface Ships: A state of the art survey. Ocean Engineering, 235, 109380. https://doi.org/10.1016/j.oceaneng.2021.109380

[6] Xu, H. and Guedes Soares, C. (2023). Review of Path-following Control Systems for Maritime Autonomous Surface Ships. Journal of Marine Science and Application, 22(2), 153-171. https://doi.org/10.1007/s11804-023-00338-6

[7] Chang, C.-H., Lai, Y.-H., Lee, Y.-H. and Chen, Y.-C. (2024). COLREG and MASS: Analytical review to identify research trends and gaps in the Development of Autonomous Collision Avoidance. Ocean Engineering, 302, 117652. https://doi.org/10.1016/j.oceaneng.2024.117652

[8] Cao, X., Wang, Z., Zhu, Y., Zhang, T., Shi, G. and Shi, Y. (2025a). A Review of Research on Autonomous Collision Avoidance Performance Testing and an Evaluation of Intelligent Vessels. Journal of Marine Science and Engineering, 13(8), 1570. https://doi.org/10.3390/jmse13081570

[9] Alamoush, A.S. and Ölçer, A.I. (2025). Maritime Autonomous Surface Ships: Architecture for Autonomous Navigation Systems. Journal of Marine Science and Engineering, 13(1), 122. https://doi.org/10.3390/jmse13010122

[10] Villa, J., Aaltonen, J. and Koskinen, K.T. (2020). Path-Following With LiDAR-Based Obstacle Avoidance of an Unmanned Surface Vehicle in Harbor Conditions. IEEE/ASME Transactions on Mechatronics, 25(4), 1812-1820. https://doi.org/10.1109/TMECH.2020.2997970

[11] Xie, Y., Nanlal, C. and Liu, Y. (2024). Reliable LiDAR-based ship detection and tracking for Autonomous Surface Vehicles in busy maritime environments. Ocean Engineering, 312, 119288. https://doi.org/10.1016/j.oceaneng.2024.119288

[12] von Brandis, A., Menges, D. and Rasheed, A. (2025). Multi-Target Tracking for Autonomous Surface Vessels Using LiDAR and AIS Data Integration. Applied Ocean Research, 154, 104348. https://doi.org/10.1016/j.apor.2024.104348

[13] Han, S., Lee, D., Kim, J. and Kim, J. (2026). A survey on maritime perception datasets and technologies for autonomous surface vessels. Intelligent Service Robotics, 19(2), Article 25. https://doi.org/10.1007/s11370-025-00689-9

[14] Cho, Y., Han, J., Kim, J., Lee, P. and Park, S.-B. (2019). Experimental validation of a velocity obstacle based collision avoidance algorithm for unmanned surface vehicles. IFAC-PapersOnLine, 52(21), 329-334. https://doi.org/10.1016/j.ifacol.2019.12.328

[15] Thyri, E.H. and Breivik, M. (2022). Partly COLREGs-compliant collision avoidance for ASVs using encounter-specific velocity obstacles. IFAC-PapersOnLine, 55(31), 37-43. https://doi.org/10.1016/j.ifacol.2022.10.406

[16] Yu, D. and Roh, M.-I. (2024). Method for anti-collision path planning using velocity obstacle and A* algorithms for maritime autonomous surface ship. International Journal of Naval Architecture and Ocean Engineering, 16, 100586. https://doi.org/10.1016/j.ijnaoe.2024.100586

[17] Li, H., Wang, X., Guedes Soares, C. and Ni, S. (2024a). A collision-avoidance decision-making scheme based on artificial potential fields and event-triggered control. Ocean Engineering, 306, 118101. https://doi.org/10.1016/j.oceaneng.2024.118101

[18] Lyu, H., Liu, W., Guo, S., Tan, G., Fu, C., Sun, X., Zhao, Y., Zhang, L. and Yin, Y. (2024). Autonomous collision avoidance method for MASSs based on precise potential field modelling and COLREGs constraints in complex sailing environments. Ocean Engineering, 292, 116530. https://doi.org/10.1016/j.oceaneng.2023.116530

[19] Zhang, M., Hao, S., Wu, D., Chen, M.-L. and Yuan, Z.-M. (2022). Time-optimal obstacle avoidance of autonomous ship based on nonlinear model predictive control. Ocean Engineering, 266, 112591. https://doi.org/10.1016/j.oceaneng.2022.112591

[20] Zhang, C., Dhyani, A., Ringsberg, J.W., Thies, F., Negenborn, R.R. and Reppa, V. (2025). Nonlinear model predictive control for path following of autonomous inland vessels in confined waterways. Ocean Engineering, 334, 121592. https://doi.org/10.1016/j.oceaneng.2025.121592

[21] Meng, J., Liu, Y., Bucknall, R., Guo, W. and Ji, Z. (2022). Anisotropic GPMP2: A Fast Continuous-Time Gaussian Processes Based Motion Planner for Unmanned Surface Vehicles in Environments With Ocean Currents. IEEE Transactions on Automation Science and Engineering, 19(4), 3914-3931. https://doi.org/10.1109/TASE.2021.3139163

[22] Mukadam, M., Dong, J., Yan, X., Dellaert, F. and Boots, B. (2018). Continuous-time Gaussian process motion planning via probabilistic inference. The International Journal of Robotics Research, 37(11), 1319-1340. https://doi.org/10.1177/0278364918790369

[23] Colledanchise, M. and Ögren, P. (2018). Behavior Trees in Robotics and AI: An Introduction. CRC Press. https://doi.org/10.1201/9780429489105

[24] Iovino, M., Scukins, E., Styrud, J., Ögren, P. and Smith, C. (2022). A survey of Behavior Trees in robotics and AI. Robotics and Autonomous Systems, 154, 104096. https://doi.org/10.1016/j.robot.2022.104096

[25] Merino-Fidalgo, S., Sánchez-Girón, C., Zalama, E., Gómez-García-Bermejo, J. and Duque-Domingo, J. (2025). Behavior tree generation and adaptation for a social robot control with LLMs. Robotics and Autonomous Systems, 194, 105165. https://doi.org/10.1016/j.robot.2025.105165

[26] Khatib, O. (1986). Real-Time Obstacle Avoidance for Manipulators and Mobile Robots. The International Journal of Robotics Research, 5(1), 90-98. https://doi.org/10.1177/027836498600500106

[27] Ester, M., Kriegel, H.-P., Sander, J. and Xu, X. (1996). A Density-Based Algorithm for Discovering Clusters in Large Spatial Databases with Noise. Proceedings of the Second International Conference on Knowledge Discovery and Data Mining, 226-231.

[28] Kalman, R.E. (1960). A New Approach to Linear Filtering and Prediction Problems. Journal of Basic Engineering, 82(1), 35-45. https://doi.org/10.1115/1.3662552

[29] Fiorini, P. and Shiller, Z. (1998). Motion planning in dynamic environments using velocity obstacles. The International Journal of Robotics Research, 17(7), 760-772. https://doi.org/10.1177/027836499801700706

[30] Fox, D., Burgard, W. and Thrun, S. (1997). The dynamic window approach to collision avoidance. IEEE Robotics and Automation Magazine, 4(1), 23-33. https://doi.org/10.1109/100.580977

[31] Fossen, T.I. (2011). Handbook of Marine Craft Hydrodynamics and Motion Control. Wiley. https://doi.org/10.1002/9781119994138

[32] Breivik, M. and Fossen, T.I. (2009). Guidance laws for autonomous underwater vehicles. In: Inzartsev, A.V. (ed.), Underwater Vehicles. IntechOpen. https://doi.org/10.5772/6696

[33] Tam, C. and Bucknall, R. (2010). Path-planning algorithm for ships in close-range encounters. Journal of Marine Science and Technology, 15(4), 395-407. https://doi.org/10.1007/s00773-010-0094-x

[34] Szlapczynski, R. and Szlapczynska, J. (2017). Review of ship safety domains: Models and applications. Ocean Engineering, 145, 277-289. https://doi.org/10.1016/j.oceaneng.2017.09.020

[35] Kuwata, Y., Wolf, M.T., Zarzhitsky, D., and Huntsberger, T.L. (2014). Safe maritime autonomous navigation with COLREGS, using velocity obstacles. IEEE Journal of Oceanic Engineering, 39(1), 110-119. https://doi.org/10.1109/JOE.2013.2254214

[36] Statheros, T., Howells, G. and McDonald-Maier, K. (2008). Autonomous ship collision avoidance navigation concepts, technologies and techniques. Journal of Navigation, 61(1), 129-142. https://doi.org/10.1017/S037346330700447X

[37] Lazarowska, A. (2017). A new deterministic approach in a decision support system for ship’s trajectory planning. Expert Systems with Applications, 71, 469-478. https://doi.org/10.1016/j.eswa.2016.11.005

[38] Perera, L.P., Carvalho, J.P. and Guedes Soares, C. (2012). Intelligent ocean navigation and fuzzy-Bayesian decision/action formulation. IEEE Journal of Oceanic Engineering, 37(2), 204-219. https://doi.org/10.1109/JOE.2012.2184949

[39] Mou, J.M., van der Tak, C. and Ligteringen, H. (2010). Study on collision avoidance in busy waterways by using AIS data. Ocean Engineering, 37(5-6), 483-490. https://doi.org/10.1016/j.oceaneng.2010.01.012

[40] Goerlandt, F. and Kujala, P. (2011). Traffic simulation based ship collision probability modeling. Reliability Engineering and System Safety, 96(1), 91-107. https://doi.org/10.1016/j.ress.2010.09.003

[41] Cyberbotics (2026). Webots User Guide. Available at: https://cyberbotics.com/doc/guide/index (accessed 29 June 2026).

[42] Li, Y., Yu, Q. and Yang, Z. (2024b). Vessel Trajectory Prediction for Enhanced Maritime Navigation Safety: A Novel Hybrid Methodology. Journal of Marine Science and Engineering, 12(8), 1351. https://doi.org/10.3390/jmse12081351

[43] Wu, B., Guo, H., Liu, Z. and Ma, W. (2026). Optimization method for collision avoidance paths of unmanned vessels under ship position prediction uncertainty. Ocean Engineering, 346, 123765. https://doi.org/10.1016/j.oceaneng.2025.123765

[44] Cao, Y., Zhang, G., Liu, Y., Zhu, S., Sun, W. and Luo, Z. (2025b). A collision avoidance decision-making system for autonomous vessels in constrained waterways based on velocity obstacles. Ocean Engineering, 342, 123013. https://doi.org/10.1016/j.oceaneng.2025.123013

\pagebreak

# Appendix A GenAI use note

This dissertation draft was developed with GenAI assistance for drafting, editing, translation support, code inspection summarization, and document generation. All technical claims, limitations, and references should be checked by the author against the final submission requirements and the university’s current GenAI declaration policy before submission.
