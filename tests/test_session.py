import csv

from robotics_sim.session import ActivitySession


def test_pause_step_and_completion_keep_world_open(tmp_path):
    session = ActivitySession(duration=0.03, dt=0.01)
    try:
        session.step()
        assert session.sim.time == 0
        session.start()
        session.step()
        session.pause()
        session.step()
        assert session.sim.time == 0.01
        session.step(single=True)
        assert session.state == "Paused"
        session.start()
        session.step()
        assert session.state == "Completed"
        assert session.world.physics.connected
        final_pose = session.robot.pose()
        session.start()
        session.step(single=True)
        assert session.robot.pose() == final_pose
        assert session.sim.steps == 3
        destination = tmp_path / "result.csv"
        session.export_csv(destination)
        with destination.open() as stream:
            assert len(list(csv.DictReader(stream))) == 3
    finally:
        session.close()
    assert not session.world.physics.connected
