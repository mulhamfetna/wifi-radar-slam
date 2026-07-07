To: [CONFIRM — Dong Huang, CMU] ; [CONFIRM — Jinsong Han, XJTU]
Subject: Commodity-WiFi 3D perception — cross-environment generalization for a mobile platform

Dear Dr. Huang and Dr. Han,

I'm [YOUR NAME], [YOUR ROLE / INSTITUTION]. The Person-in-WiFi line of work — the 2019 ICCV paper and the
CVPR 2024 3D extension — is central to our project, because it demonstrated dense human perception from
*commodity* 802.11 CSI rather than purpose-built radios. That commodity-hardware result is what makes an
affordable on-vehicle WiFi sensor plausible at all.

We are trying to take commodity-CSI perception onto a moving car to build a 3D scan of the surroundings
for SLAM. The limitation your papers report so honestly — the sharp drop in untrained environments (mIoU
0.12, only partly recovered by domain adaptation) and the difficulty of cross-location generalization —
is precisely our central challenge, because on a moving vehicle *every frame is effectively a new
environment*.

If you have time, I'd greatly value your thoughts on:

1. Which factor dominated the cross-environment degradation you observed — antenna geometry, multipath
   specificity, or training-data diversity — and which you'd attack first?
2. Whether you see continual/online adaptation as a realistic route for a receiver that never revisits
   the same scene.

Thank you for work that made the commodity-hardware case so convincingly.

Best regards,
[YOUR NAME]
[YOUR ROLE / INSTITUTION]
[EMAIL / PHONE]
