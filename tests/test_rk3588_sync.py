"""Mac-side RK3588 integration guards. No live DDS or board connection."""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'dds_swarm_fusion'))
import field_simulator as sim
from evaluate_field_sim import evaluate
from field_sim_web import Controller
from company_input_node import CompanyInputNode
from dds_fusion_node import FusionDdsNode
from fusion import Fusion
from replay_radar_log import MemoryBus,replay


class RkSyncTests(unittest.TestCase):
    def test_existing_field_default_preserves_low_confidence_reports(self):
        with patch.dict(os.environ,{},clear=True):
            self.assertEqual(Fusion({},.5).min_confidence,.05)
            self.assertEqual(FusionDdsNode(MemoryBus()).min_confidence,.05)
            r=replay(ROOT/'samples/radar_3s.log',scenario=ROOT/'configs/equipment.yaml',
                     radar_input_mode='device_tracks',azimuth_convention='north_cw')
        self.assertEqual(r['frames_with_fused_output'],21)
        self.assertEqual(r['frames_with_external_output'],21)
        self.assertEqual(r['real_points'],0)

    def test_explicit_confidence_overrides_env_and_invalid_rejected(self):
        with patch.dict(os.environ,{'SB_FUSION_MIN_CONFIDENCE':'.05'}):
            node=FusionDdsNode(MemoryBus(),min_confidence=.2)
            node.sync_fusion({'radar_1':dict(id='radar_1',kind='radar',pos=[0,0,0])})
            self.assertEqual(node.fusion.min_confidence,.2)
            self.assertEqual(node.fusion_eo.min_confidence,.2)
        for value in ('nan','inf','-1','2','not-a-number'):
            with self.subTest(value=value),patch.dict(os.environ,{'SB_FUSION_MIN_CONFIDENCE':value}):
                with self.assertRaises(ValueError):FusionDdsNode(MemoryBus())

    def test_generated_test_discovers_only_received_devices(self):
        with tempfile.TemporaryDirectory() as td:
            dest=Path(td)/'run';sim.generate_run(dest,sim.load_settings(duration_s=15),'custom')
            node=CompanyInputNode(dest/'scenario.yaml',bus=MemoryBus())
            self.assertEqual(node.sensors_info,{})
            sid,info=node._sensor('eo',71112)
            self.assertEqual(sid,'eo_71112');self.assertIn('pos',info)
            self.assertEqual(list(node.sensors_info),['eo_71112'])

    def test_offline_evaluation_pins_strict_policy_and_reports_it(self):
        with tempfile.TemporaryDirectory() as td,patch.dict(os.environ,{'SB_FUSION_MIN_CONFIDENCE':'.05'}):
            dest=Path(td)/'run'
            sim.generate_run(dest,sim.load_settings(duration_s=15),'radar-extrapolation')
            r=evaluate(dest)
            self.assertEqual(r['formal_output_min_confidence'],.2)
            self.assertTrue(r['checks']['no_formal_localization'])
            self.assertEqual(r['multi_target_metrics']['recall'],0)
            self.assertEqual(os.environ['SB_FUSION_MIN_CONFIDENCE'],'.05')

    def test_ui_command_matches_evaluation_and_uses_current_root(self):
        with tempfile.TemporaryDirectory() as td:
            ctl=Controller(td)
            try:
                run='run-20261023-000000-12345678'
                sim.generate_run(Path(td)/run,sim.load_settings(duration_s=15),'custom')
                command=ctl.details(run)['launch_command']
                self.assertIn('SB_FUSION_MIN_CONFIDENCE=0.2 SB_EO_CONFIG=',command)
                self.assertIn(str(ROOT),command)
                self.assertNotIn('/home/dgx/',command)
            finally:ctl.close()

    def test_local_official_dds_viewer_and_diagnostic_context_preserved(self):
        from dds_ws_gateway import Gateway,create_app
        gateway=Gateway(bus=MemoryBus())
        self.assertIn('official_fusion',gateway.diagnostics_snapshot())
        app=create_app(bus=MemoryBus())
        self.assertIn('/api/official-fusion',{r.path for r in app.routes})
        bus=MemoryBus();node=CompanyInputNode(ROOT/'configs/equipment.yaml',bus=bus)
        node.publish_diagnostics(force=True)
        from company_dds import Topics
        diagnostic=bus.messages[Topics.DIAGNOSTICS.name]
        self.assertEqual(diagnostic['scenario_path'],str(ROOT/'configs/equipment.yaml'))
        # Diagnostics must follow the selected site's reference, not the old demo origin.
        self.assertEqual(diagnostic['origin'],node.config.coordinates.origin)


if __name__=='__main__':unittest.main()
