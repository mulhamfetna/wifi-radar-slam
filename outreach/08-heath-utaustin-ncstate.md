To: rheath@utexas.edu
Subject: 802.11ad-based radar for automotive sensing — a WiFi-illuminator SLAM question

Dear Professor Heath,

I'm [YOUR NAME], [YOUR ROLE / INSTITUTION]. Your work on IEEE 802.11ad-based radar — the joint vehicular
communication–radar system reusing the 60 GHz preamble's Golay sequences as a radar waveform, and the
follow-on V2I ISAR imaging paper — is central to a decision we're facing on a WiFi-as-radar project for
automotive SLAM.

We're building a sensor that turns WiFi into a 3D scan of a vehicle's surroundings as a radar substitute
for SLAM. Our literature analysis makes the bandwidth ceiling stark: commodity sub-7 GHz WiFi is limited
to roughly 1–4 m range resolution (ΔR = c/2B), while your 802.11ad approach reaches ~8.5 cm — which is
why the 60 GHz path is so attractive despite its range and penetration costs.

Two questions where your perspective would be invaluable:

1. Your 802.11ad-radar results are, as I read them, analytical / CRB / Monte-Carlo under idealized
   full-duplex self-interference cancellation. In the years since, how close has hardware come to
   realizing them — is the self-interference-cancellation assumption the main gap between the analysis
   and a deployable 60 GHz ISAC radar?
2. For a SLAM use case that needs 3D scene structure (not just target detection), would you expect the
   802.11ad preamble waveform to support dense mapping, or is a dedicated sensing waveform / 802.11bf DMG
   still the more realistic route?

I'd be grateful for any pointers, and would welcome a short call if useful. Thank you for work that has
essentially defined this corner of ISAC.

Best regards,
[YOUR NAME]
[YOUR ROLE / INSTITUTION]
[EMAIL / PHONE]
