# Email to the IEEE IoT-J Editor — self-reported erratum

> **How to use:** fill in the bracketed fields (manuscript ID, editor name) and send from
> contact@mulhamfetna.com. Attach the corrected manuscript. Send this **before** the paper
> goes out for review results, and do not wait for reviewer comments.

---

**Subject:** Self-reported correction to manuscript [MANUSCRIPT ID] — one reported value does not reproduce

Dear [Editor's name / Dear Editor],

I am writing to report, on my own initiative, an error I found in my recently submitted
manuscript **[MANUSCRIPT ID], "Ambient WiFi as a Radar Replacement for Automotive SLAM: A
Physics-Based Feasibility Study."**

While regenerating results for a follow-up study, I re-ran the released artifact against the
submitted manuscript and found that **one reported value does not reproduce**. I would
rather bring this to you now than have it discovered during review.

### The error

The manuscript reports that joint 2-D (delay–angle) MUSIC lifts realistic-CSI localization to
**ATE = 0.027 m**, described as "matching" the 0.045 m oracle. Re-running the submitted
code with the submitted configuration (`configs/controlled_music_joint.yaml`) gives:

| Seed | 42 | 1 | 2 | 3 | 4 | 5 |
|------|----|----|----|----|----|----|
| ATE (m) | 0.143 | 0.108 | 0.056 | 0.089 | 0.092 | 0.097 |

**Mean 0.098 ± 0.028 m.** The reported 0.027 m lies below the minimum of every seed, so this
is not run-to-run variance. The associated map metrics differ similarly (reported Chamfer
4.1 m / completeness 3.5 m; actual ≈6.0 m / ≈9.5 m).

I believe the figure was recorded from an earlier state of the code during development and
was not re-verified against the final committed version before submission. It is an error of
carelessness on my part, not of intent, and I regret it.

### Scope — the rest of the paper reproduces

I audited every headline result against the released artifact (5 seeds each). The paper's
principal contributions are unaffected:

| Result | Reported | Reproduced | |
|---|---:|---:|---|
| Controlled-scene oracle map: ATE / accuracy / IoU | 0.045 / 0.25 / 0.79 | 0.049 ± 0.027 / 0.248 ± 0.003 / 0.791 | ✔ |
| Street-scene oracle map: ATE / accuracy / IoU | 0.116 / 0.30 / 0.077 | 0.104 ± 0.041 / 0.309 ± 0.010 / 0.077 | ✔ |
| 60 GHz / 16-antenna null result (mapping is not resolution-limited) | — | reproduces | ✔ |
| **Realistic joint-MUSIC ATE** | **0.027** | **0.098 ± 0.028** | ✘ |

The qualitative finding also survives: joint 2-D MUSIC still improves realistic ATE by
roughly **7×** over sorted 1-D pairing (0.73 m → 0.098 m). What is **not** supported is the
stronger claim that it reaches *oracle-quality* localization. The corrected manuscript states
that it **approaches but does not match** the oracle (0.098 ± 0.028 m vs 0.049 ± 0.027 m),
and now reports WiFi results as **mean ± standard deviation over five seeds** rather than a
single run, which is the more honest statistic for a stochastic pipeline.

### Two further corrections included

1. **Learned path discriminator (F1 ≈ 0.9).** One of its input features is the arrival
   *elevation*, which the single-ULA delay–azimuth front-end described in the paper does not
   estimate. The reported F1 is therefore optimistic. Retrained on only the features that
   front-end can actually measure, held-out F1 is 0.00–0.45. I have removed the claim that
   the discrimination is straightforwardly learnable from commodity CSI.

2. **Interpretation of the mapping floor.** Follow-up analysis indicates the realistic-mapping
   floor is dominated not by path discrimination, as the paper argues, but by phantom
   detections (≈89 % of MUSIC detections correspond to no real propagation path) and an
   estimator range bias. The corrected manuscript softens the interpretation accordingly; the
   *empirical* mapping results are unchanged.

### What I am providing

A corrected manuscript with these changes marked is attached. The public artifact
(repository and Zenodo record) has been annotated with an erratum documenting the defect,
the seed-level audit, and the correction, so that anyone reproducing the work sees the
corrected values.

I am happy to proceed however you judge best — whether that is replacing the manuscript
under review, treating this as a revision, or withdrawing and resubmitting. Please let me
know what you would prefer.

I apologise for the error and for any additional work it causes you and the reviewers.

Sincerely,
Mulham Fetna
ORCID 0009-0006-4432-798X
contact@mulhamfetna.com
