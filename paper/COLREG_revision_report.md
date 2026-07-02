# COLREG thesis revision report

Date: 29 June 2026  
Protected source: `COLREG_Compliant_ASV_Navigation_Yida_Zhu_Final_Cited_Flowchart.docx`  
Protected backup: `backups/COLREG_Compliant_ASV_Navigation_Yida_Zhu_Final_Cited_Flowchart_pre_apf_ekf_20260629.docx`  
Revised deliverable: `COLREG_Compliant_ASV_Navigation_Yida_Zhu_Final_Cited_Flowchart_APF_EKF.docx`

## Completed changes

1. Added claim-level Harvard author-year citations to statements about maritime perception, vessel safety domains, prediction, encounter responsibilities, APF, reactive and predictive planning, CPA/TCPA/DCPA, and validation practice.
2. Added a grayscale four-panel COLREG schematic covering:
   - Rule 13 overtaking;
   - Rule 14 head-on;
   - Rules 15-16 crossing from the give-way vessel's perspective;
   - Rule 17 crossing from the stand-on vessel's perspective.
3. Added editable Word pseudocode derived directly from `select_colreg_strategy()` and `compute_apf_control()` in `src/laptop.py`.
4. Added a grayscale engineering flowchart that visualizes the pseudocode's candidate-evaluation, risk-ranking, and APF-dispatch branches.
5. Renumbered the existing experimental figures to Figures 4-19 after inserting the COLREG schematic, pseudocode flowchart, and APF-generation schematic as Figures 1-3.
6. Corrected reference metadata and applied hanging indents to the reference list.
7. Changed heading and caption text to black in the revised output.
8. Added the APF and EKF principles as numbered equations, including the implemented five-state CTRV transition, nonlinear EKF prediction/update, Joseph covariance update, classical APF potential, PC1/PC2 ellipse level, size-aware repulsive weight, resultant force, and bounded heading command.
9. Corrected the earlier description of the obstacle predictor from constant velocity to the five-state CTRV model present in `ObstacleEKF`.
10. Added a grayscale, code-grounded schematic showing potential-field generation, the PC1/PC2 safety ellipse, an EKF-predicted virtual obstacle, and the controller's force decomposition.
11. Applied Harvard author-year citations directly around the new principles and figure explanation: `(Kalman, 1960; Xie et al., 2024)` for filtering/tracking and `(Khatib, 1986; Lyu et al., 2024; Li et al., 2024a)` for APF principles and maritime constraints.

## Citation corrections

- Breivik and Fossen (2009): corrected source to *Underwater Vehicles* and DOI to `10.5772/6696`.
- Tam and Bucknall (2010): corrected DOI suffix to `10.1007/s00773-010-0094-x`.
- Lazarowska (2017): corrected DOI to `10.1016/j.eswa.2016.11.005`.
- Perera et al. (2012): corrected DOI to `10.1109/JOE.2012.2184949`.
- Goerlandt and Kujala (2011): corrected DOI to `10.1016/j.ress.2010.09.003`.
- von Brandis et al.: aligned the journal issue year with the 2025 publication record.
- Han et al. (2026) and Meng et al. (2022): added missing article/page information.

The DOI-derived bibliography contains 40 entries and passes the citation-management validator with zero errors, warnings, or duplicates. The non-DOI IMO convention, original DBSCAN conference paper, and Webots guide were checked against authoritative or publisher sources. The von Brandis et al. record was also confirmed through the publisher and DOAJ after CrossRef temporarily rate-limited the final batch request.

## Evidence boundaries

- The COLREG drawing shows responsibilities and typical initial actions; it does not claim that COLREGs define exact software sectors, TCPA/DCPA thresholds, or a mandatory overtaking side.
- The pseudocode reproduces the code's selection order and controller dispatch, but omits exception handling, logging plumbing, and CSV/snapshot patching.
- No new performance claims, success rates, or experimental runs were added.
- The standard EKF/APF equations are identified as established theory; project-specific equations and variable mappings are explicitly tied to functions in `src/laptop-crossing.py`.

## QA status

- Original DOCX preserved and copied before structural edits.
- Revised DOCX contains 537 paragraphs, 14 numbered equations, 19 figures, and one editable algorithm table.
- Structural QA confirms continuous equation numbering `(3.1)`-`(3.14)`, continuous figure numbering `1`-`19`, all required APF/EKF source references, and no unresolved build markers.
- Accessibility audit: zero high, medium, or low findings after adding descriptive image alt text and marking the algorithm header row.
- Reference validator: 40 valid DOI-derived entries; zero errors, warnings, or duplicates.
- Full DOCX-to-PNG page rendering could not be completed because LibreOffice is not installed and Microsoft Word's hidden PDF export stalled. The new figure was inspected directly at full resolution, and the DOCX passed structural OOXML and accessibility checks. A final local page-by-page visual pass in Word is still advisable before submission.
