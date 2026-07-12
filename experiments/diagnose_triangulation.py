import numpy as np
from wifi_radar_slam.config import load_config
from wifi_radar_slam.scene.builder import build_scene
from wifi_radar_slam.channel.simulator import simulate_csi
from wifi_radar_slam.sensing.frontend import extract_detections
from wifi_radar_slam.geometry import velocity_from_poses
from wifi_radar_slam.slam.particle_filter import run_slam, _triangulate_bistatic
from wifi_radar_slam.sensing.oracle import extract_oracle_detections

for scene, cfgp in [("controlled_wall","configs/controlled_music_joint.yaml"),
                    ("street_canyon","configs/street_metal_music.yaml")]:
    cfg=load_config(cfgp); built=build_scene(cfg)
    gt, gt_xy = built.trajectory, built.ground_truth_map[:,:2]
    vel=velocity_from_poses(gt,cfg.trajectory.timestep_s)
    csi=simulate_csi(built,cfg.rf,cfg.snr_db,np.random.default_rng(cfg.seed))
    dets=extract_detections(csi,cfg.rf,n_paths=3,world_aoa=cfg.world_aoa,joint=cfg.joint_estimation)
    est,_=run_slam(dets,built.ap_positions,vel,cfg.trajectory.timestep_s,
                   np.random.default_rng(0),init_pose=gt[0],
                   map_min_support=cfg.map_min_support,map_min_excess_m=cfg.map_min_excess_m)
    def errs(dlist, poses, tag):
        e=[]; nfail=0
        for f in range(len(dlist)):
            for (pl,aoa,ap_i) in np.asarray(dlist[f]).reshape(-1,3):
                ap=np.asarray(built.ap_positions[int(ap_i)])[:2]
                r=_triangulate_bistatic(poses[f][:2],ap,pl,aoa)
                if r is None: nfail+=1; continue
                e.append(np.min(np.linalg.norm(gt_xy-r,axis=1)))
        e=np.array(e)
        if len(e)==0: print(f"  {tag}: no valid triangulations"); return
        print(f"  {tag}: n={len(e)} fail={nfail} | err_to_facade  median={np.median(e):.2f}m  "
              f"p25={np.percentile(e,25):.2f}  p10={np.percentile(e,10):.2f}  min={e.min():.2f}")
        for t in (1.0,2.0,3.0,5.0):
            print(f"      within {t:.0f}m: {100*(e<=t).mean():.1f}%")
    print(f"=== {scene} ===")
    errs(dets, est, "REALISTIC MUSIC (est pose)")
    # ORACLE upper bound: Sionna's TRUE single-scatter delay/AoA, same triangulation
    odets=extract_oracle_detections(built,cfg.rf,np.random.default_rng(0))
    errs(odets, gt, "ORACLE paths (gt pose)")
