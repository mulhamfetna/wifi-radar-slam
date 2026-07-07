To: christian.gentner@dlr.de
Subject: Channel-SLAM on a fast-moving vehicular receiver — a question about robustness

Dear Dr. Gentner,

I'm [YOUR NAME], [YOUR ROLE / INSTITUTION]. Your Channel-SLAM line of work — the multipath-component-
as-virtual-transmitter formulation, and the pedestrian LTE positioning paper in Mobile Information
Systems — is the back-end we are hoping to build on. We are developing a WiFi-based passive-radar sensor
mounted on a car that produces a 3D scan of its surroundings for SLAM, and your virtual-transmitter
landmark model (no prior map, no known reflector positions) fits that problem almost perfectly.

My main question is about regime transfer. Channel-SLAM was demonstrated for pedestrian / relatively
slow motion. For an automotive receiver — much higher speed, rapidly changing multipath geometry,
moving outdoor clutter — do you expect the virtual-transmitter estimation (and the Rao-Blackwellized
particle filter) to remain stable, or would the association of persistent virtual transmitters across
frames become the bottleneck?

Any pointers to work in your group on higher-dynamics receivers, or advice on where the formulation is
most fragile, would be enormously helpful before we commit to an implementation path.

Thank you for foundational work that we expect to lean on heavily.

Best regards,
[YOUR NAME]
[YOUR ROLE / INSTITUTION]
[EMAIL / PHONE]
