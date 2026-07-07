To: dk@mit.edu
Subject: Dense 3D reconstruction from RF — moving from indoor FMCW to vehicular WiFi

Dear Professor Katabi,

I'm [YOUR NAME], [YOUR ROLE / INSTITUTION]. The RF-Pose and RF-Pose3D work from your group is a big part
of why our project seems feasible: it established that RF signals in the WiFi band carry enough
information to reconstruct dense human structure, including through walls and in 3D.

We're pursuing a related but shifted target — an on-vehicle sensor that turns ambient WiFi into a 3D
scan of the *outdoor* surroundings for SLAM. I'm careful to distinguish your purpose-built WiFi-band
FMCW radio from commodity 802.11 CSI, and part of our open question is exactly how much of the dense-
reconstruction capability survives when you drop from an engineered FMCW waveform to whatever ambient
WiFi provides.

If you have a moment, I'd value your intuition on:

1. In your experience, how much of RF-Pose's reconstruction quality came from the FMCW waveform/bandwidth
   specifically, versus the cross-modal learning setup? I.e. how badly would commodity-CSI bandwidth hurt
   the result?
2. Whether you'd expect the through-occlusion advantage of RF to matter (or become a liability via
   uncontrolled multipath) on a fast-moving outdoor platform.

Thank you for work that opened up this whole direction.

Best regards,
[YOUR NAME]
[YOUR ROLE / INSTITUTION]
[EMAIL / PHONE]
