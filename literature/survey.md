# Literature Survey

## Scope-derived foundation

The Scoping Study defines the intended architecture as perception, interpretable rule-based decision making, reactive APF and longer-horizon planning in Webots. Its cited surveys consistently identify three linked functions: collision-risk detection, encounter/rule decision, and path replanning.

## Verified core sources

1. International Maritime Organization (1972), *Convention on the International Regulations for Preventing Collisions at Sea*. Rules 5-8 govern lookout, safe speed, collision risk and avoidance action; Rules 13-17 govern overtaking, head-on, crossing, give-way and stand-on conduct. https://www.imo.org/en/about/conventions/pages/colreg.aspx
2. Hu, L. et al. (2022), *A review on COLREGs-compliant navigation of autonomous surface vehicles: From traditional to learning-based approaches*, Journal of Automation and Intelligence, 1(1), 100003. https://doi.org/10.1016/j.jai.2022.100003
3. Burmeister, H.-C. and Constapel, M. (2021), *Autonomous Collision Avoidance at Sea: A Survey*, Frontiers in Robotics and AI, 8, 739013. https://doi.org/10.3389/frobt.2021.739013
4. Iovino, M. et al. (2022), *A survey of Behavior Trees in robotics and AI*, Robotics and Autonomous Systems, 154, 104096. https://doi.org/10.1016/j.robot.2022.104096
5. Xie, Y., Nanlal, C. and Liu, Y. (2024), *Reliable LiDAR-based ship detection and tracking for Autonomous Surface Vehicles in busy maritime environments*, Ocean Engineering, 312, 119288. https://doi.org/10.1016/j.oceaneng.2024.119288
6. Cho, Y. et al. (2019), *Experimental validation of a velocity obstacle based collision avoidance algorithm for unmanned surface vehicles*, IFAC-PapersOnLine, 52(21), 329-334. https://doi.org/10.1016/j.ifacol.2019.12.328
7. Meng, J. et al. (2022), *Anisotropic GPMP2: A Fast Continuous-Time Gaussian Processes Based Motion Planner for Unmanned Surface Vehicles in Environments With Ocean Currents*, IEEE Transactions on Automation Science and Engineering. https://doi.org/10.1109/TASE.2021.3139163
8. Chang, C.-H. et al. (2024), *COLREG and MASS: Analytical review to identify research trends and gaps in the Development of Autonomous Collision Avoidance*, Ocean Engineering, 302, 117652. https://doi.org/10.1016/j.oceaneng.2024.117652
9. Khatib, O. (1986), *Real-Time Obstacle Avoidance for Manipulators and Mobile Robots*, The International Journal of Robotics Research, 5(1), 90-98. https://doi.org/10.1177/027836498600500106. Foundational source for attractive/repulsive artificial potential fields and negative-gradient motion.
10. Kalman, R.E. (1960), *A New Approach to Linear Filtering and Prediction Problems*, Journal of Basic Engineering, 82(1), 35-45. https://doi.org/10.1115/1.3662552. Foundational prediction-correction and covariance framework from which the project EKF is derived.
11. Lyu, H. et al. (2024), *Autonomous collision avoidance method for MASSs based on precise potential field modelling and COLREGs constraints in complex sailing environments*, Ocean Engineering, 292, 116530. https://doi.org/10.1016/j.oceaneng.2023.116530. Supports the maritime extension of potential fields with target-vessel and COLREG constraints.

## Gap relevant to this dissertation

The literature supports modular, explainable rule encoding and predictive collision metrics, but it also warns that most published systems cover only overtaking, head-on and crossing, assume high-quality perception, and lack harmonised validation. The present project is therefore best positioned as an implementation-and-validation study with explicit failure analysis, not as proof of universal COLREG compliance.

For the APF/EKF theory revision, CrossRef metadata and publisher/search records were checked on 29 June 2026. The thesis uses Harvard author-year citations at the claim and equation-explanation level: `(Khatib, 1986; Lyu et al., 2024)` for potential-field principles and maritime constraints, and `(Kalman, 1960; Xie et al., 2024)` for filtering and maritime target tracking.
