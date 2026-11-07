# V20.1 Single-sphere acquisition hardening

This revision addresses two real-machine requirements identified after long-term use of the LLT2950-50/BL.

1. **Robust sphere arc extraction.** The rectangular X/Z ROI is now only a coarse candidate window. A deterministic circle RANSAC rejects background, fixture edges and isolated invalid/boundary returns. Only RANSAC inliers are passed to the 14-D calibration solver. The fit report exposes candidate count, inlier count/ratio, arc span and inlier RMS.
2. **Static calibration uses software/internal trigger and full measuring field.** Starting the single-sphere calibration preview temporarily restores the project's `Large` field preset (`ROI1_PRESET=0`), then starts the existing SoftwareInternal profile stream. The field change is not stored in user mode. The software ROI does not modify sensor measuring range.
3. **Full-field rate guard.** If exposure+idle were previously configured for a cropped high-speed preset, the calibration preview increases idle as necessary to stay below the full-field maximum profile rate.
4. **Pose planning decoupled from extraction ROI.** D-optimal planning continues to target the conservative central full-field region (Z 80--110 mm, |X| <= 18 mm), regardless of how tightly the operator draws the RANSAC candidate ROI.
