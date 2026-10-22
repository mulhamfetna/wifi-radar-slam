# Submission guide — Pervasive and Mobile Computing (Elsevier)

Step-by-step procedure for submitting `manuscript/` to *Pervasive and Mobile Computing* (PMC).
Status and decisions live in `../PROJECT.md`; this file is only the procedure.

> **Submitted 2026-10-22** through Elsevier's new submission service (it replaced the Editorial
> Manager form for this journal at the time). Tracking and revisions happen in the journal account
> linked from the confirmation email. Steps 4.x below describe the older Editorial Manager flow.

## Links

| What | Link |
|---|---|
| Submission portal (Editorial Manager) | https://www.editorialmanager.com/pmc/ |
| Guide for authors | https://www.sciencedirect.com/journal/pervasive-and-mobile-computing/publish/guide-for-authors |
| Declaration (competing interests) tool | https://declarations.elsevier.com/ |
| Elsevier: author guide to Editorial Manager | https://www.elsevier.support/publishing/answer/author-guide-to-editorial-manager |
| Elsevier: video guide to the declaration tool | https://www.elsevier.support/publishing/answer/video-guide-using-the-declaration-tool-in-editorial-manager |
| Data/code archive (cited in the paper) | https://doi.org/10.5281/zenodo.23560036 |
| Your ORCID | https://orcid.org/0009-0006-4432-798X |

The guide-for-authors page sits behind a CAPTCHA for automated readers; open it in your own
browser. Field and button names in Editorial Manager can differ slightly from those below.

---

## Step 0 — Before you start (10 minutes)

1. Read `manuscript/main.pdf` end to end. You are signing off every claim, including the
   withdrawn-claims paragraph in the Discussion.
2. Confirm the generative-AI declaration (last sections of `main.tex`) is accurate in your words.
   It is your declaration; edit it if needed.
3. Make sure your ORCID record is public and shows your current affiliation.
4. Decide now: **subscription route (no fee)** or open access (article publishing charge after
   acceptance). Recommended: subscription. Openness is already provided by the Zenodo archive and
   the optional SSRN preprint. If you choose open access, check the journal's open-access page for
   the current charge and waiver eligibility before accepting.

## Step 1 — Build the upload package (2 minutes)

```sh
cd /mnt/data/projects/wifi-radar
bash manuscript/make-submission.sh
```

It rebuilds the paper, checks that the LaTeX source compiles in an empty directory, and writes
`manuscript/submission/` (gitignored):

| File | Upload as |
|---|---|
| `01-latex-source.zip` | LaTeX source (main.tex, refs.bib, main.bbl, elsarticle.cls, elsarticle-num.bst, figures/) |
| `02-manuscript.pdf` | Manuscript PDF, if the system asks for one alongside the source |
| `03-highlights.docx` (or `.txt`) | Highlights |
| `04-cover-letter.pdf` (or `.docx`) | Cover letter |
| `05-Figure_1.pdf` … `05-Figure_4.pdf` | Figures, one file each, in this order |
| `declaration.docx` (Step 3) | Declaration of interest statement |

## Step 2 — Create or open your Editorial Manager account (5 minutes)

1. Go to https://www.editorialmanager.com/pmc/.
2. Choose **Register** if you have no account for this journal (or **Login** if you do). Elsevier
   journals can also offer sign-in with an Elsevier account; either works.
3. Use **contact@mulhamfetna.com** as your email, so it matches the manuscript's title page.
4. In your profile, link your **ORCID** (0009-0006-4432-798X) when offered.
5. Affiliation: *Department of Mechatronics Engineering, University of Aleppo, Aleppo, Syria*.

## Step 3 — Generate the declaration of interest statement (5 minutes)

1. Open https://declarations.elsevier.com/ (also reachable from the "Attach files" step).
2. Sign in with your email and the one-time passcode it sends.
3. Enter the manuscript title and yourself as the only author.
4. For competing interests choose **"I have nothing to declare"** (this matches the manuscript).
5. Download the Word file and save it as `manuscript/submission/declaration.docx`.

## Step 4 — Submit in Editorial Manager (30–45 minutes)

Log in as **Author**, then **Submit New Manuscript**. Work through the steps in order.

**4.1 Article type.** Choose the original research article option (the guide lists "Original
research articles"; the menu may call it "Research Paper" or "Full Length Article"). Do **not**
choose Fast Communication or a special-issue (VSI) type.

**4.2 Attach files.** Upload each file from Step 1 and pick its item type:

| Item type (approximate name) | File |
|---|---|
| Manuscript / LaTeX source file | `01-latex-source.zip` (or upload its contents individually if a zip is refused; then `main.tex` is the manuscript and the rest are "LaTeX supplementary/source" items) |
| Manuscript PDF (if requested) | `02-manuscript.pdf` |
| Highlights | `03-highlights.docx` |
| Cover letter | `04-cover-letter.pdf` |
| Figure | `05-Figure_1.pdf` to `05-Figure_4.pdf`, in order |
| Declaration of interest statement | `declaration.docx` |

Do not upload a graphical abstract (optional; none was prepared).

**4.3 General information / classifications.** Pick the closest topics from the journal's list:
*Positioning, Localization and Tracking Technologies*; *Internet of People and Internet of
Vehicles*; *Enabling Pervasive Communication Technologies (wireless LANs …)*; *Performance
Evaluation of Pervasive and Mobile Computing Systems*.

**4.4 Additional information.** Answer each question; the expected answers:

| Question (typical) | Answer |
|---|---|
| Previously published? | No. |
| Under consideration elsewhere? | No. |
| Previously submitted to another journal? | Yes — an earlier, substantially different version was rejected by *IEEE Internet of Things Journal* (2026) and a consolidated version by *IEEE Access* (2026, without review). Disclosed in the cover letter. |
| Preprints / similarity | Earlier unpublished drafts are archived publicly (Zenodo v0.7.1, 10.5281/zenodo.21265865); disclosed in the cover letter. |
| Generative AI used? | Yes — declaration included as a section before the references. |
| Data availability | Data and code deposited: https://doi.org/10.5281/zenodo.23560036 |
| Funding | None. |
| SSRN preprint? | Your choice. Yes posts the manuscript publicly after desk review; it does not affect the decision. |

**4.5 Comments.** Paste a one-line note: *"Cover letter attached; it discloses the relationship to
the author's earlier unpublished work and its corrections."*

**4.6 Manuscript data.** These fields are typed in separately from the files. Paste from the
"Paste-ready fields" section below:
- Full title
- Abstract (plain text, 249 words)
- Keywords (one per line)
- Authors: Mulham Fetna, corresponding, with ORCID
- Funding: none

**4.7 Suggested and opposed reviewers.** Optional unless the form requires it. Candidates drawn
from authors the paper cites (verify current affiliation and email on their institutional pages,
and suggest only people with whom you have no collaboration or conflict):
Erik Leitinger and Klaus Witrisal (Graz University of Technology), Christian Gentner (German
Aerospace Center, DLR), Fredrik Tufvesson (Lund University). Opposed reviewers: none.

**4.8 Build PDF for approval.** Click **Build PDF for Approval**. Editorial Manager compiles the
LaTeX itself; wait for the build (minutes), then open the generated PDF and check:
all 17 pages present, four figures, three tables, references numbered with DOIs, no "??"
cross-references. If the build fails, see Troubleshooting.

**4.9 Approve and submit.** In *Submissions Waiting for Author's Approval*, click
**Approve Submission**. You should receive a confirmation email with a manuscript number.

## Step 5 — After submitting

1. Record the manuscript number and date in `PROJECT.md` (status and decision log), commit, push.
2. Track status under *Submissions Being Processed*. PMC uses single-anonymized review: editors
   assess first; if sent out, at least two reviewers.
3. **Desk reject:** you will be told quickly; the journal may offer Elsevier's Article Transfer
   Service to another Elsevier title — consider it.
4. **Revise:** answer every point in a response letter; keep all numbers regenerated from
   `experiments/` and update `PROJECT.md` in the same commit as any result change.
5. **Accept:** choose the publishing agreement (subscription unless you decided otherwise);
   proofs must be returned within two days.

## Paste-ready fields

**Title**

Can Ambient WiFi Support Vehicular Multipath SLAM? Virtual-Anchor Localization, Commodity
Front-End Limits, and a Delay-Convention Pitfall in Ray-Traced Evaluation

**Abstract (plain text, 249 words)**

Ambient WiFi comes from existing infrastructure and reaches hardware a vehicle already carries, making it an attractive low-cost aid for vehicular simultaneous localization and mapping (SLAM). We ask what commodity channel state information (CSI) can actually contribute, using ray-traced channels (Sionna RT) in three outdoor scenarios, wheel-grade odometry as the baseline that any aid must beat, and a virtual-anchor FastSLAM back-end in which each wall reflection is represented by the access point mirrored in that wall. With exact multipath parameters the geometry works: WiFi reduces odometry drift by 66–79 % and, beside a single reflecting wall, outperforms a simulated spinning LiDAR on average. With a commodity front-end it does not: joint delay–angle multiple signal classification (MUSIC) on a four-element linear array returns only about 15 % facade reflections, about a third of detections match no propagation path, and a further 19–34 % are real paths reported at their front/back mirror angle. These errors persist across frames, are confirmed as false anchors, and make WiFi-aided SLAM worse than dead reckoning in every scenario; the share of unexplained detections stays at a third or more from 20 to 160 MHz. We also report a delay-convention pitfall in ray-traced CSI studies: channel frequency responses whose delays are referenced to the first arrival, as on real hardware, were read as absolute path lengths in our earlier analyses. This produced a spurious several-metre range bias and an inflated phantom rate, which vanish once excess delay is modelled. Code, cached channels, and firmware are open.

**Keywords**

WiFi sensing
channel state information
multipath-assisted SLAM
virtual anchors
vehicular localization
ray tracing

**Highlights** (also in `highlights.txt`)

- Exact WiFi multipath parameters cut vehicle odometry drift by 66-79 %.
- A 4-element MUSIC front-end returns only ~15 % facade reflections.
- Persistent false anchors make realistic WiFi SLAM worse than dead reckoning.
- Ambient-WiFi aiding needs access-point positions known to sub-metre accuracy.
- Reading first-arrival-referenced CSI delays as absolute creates a false range bias.

**Funding statement**

This research did not receive any specific grant from funding agencies in the public,
commercial, or not-for-profit sectors.

**Data availability**

All code, configurations, result files, and ESP32 firmware used in this study are archived on
Zenodo as release v1.0.0 (https://doi.org/10.5281/zenodo.23560036) under AGPL-3.0.

## Troubleshooting

| Problem | Fix |
|---|---|
| PDF build fails on the class | Upload `elsarticle.cls` and `elsarticle-num.bst` explicitly as LaTeX source items |
| References missing in the built PDF | Upload `main.bbl` (in the zip); Editorial Manager does not always run BibTeX |
| Figures missing | Keep them inside a `figures/` folder, or re-upload each as an individual item with the same file names (`localization.pdf`, `frontend_decomposition.pdf`, `maps_controlled.pdf`, `robustness.pdf`) |
| Zip refused | Upload the files from `manuscript/submission/latex-source/` one by one |
| Abstract field rejects symbols | Replace "–" with "-" and "%" spacing as needed |
