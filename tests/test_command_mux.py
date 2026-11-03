import unittest
from go2_rl_controller.command_mux import CommandGate


class CommandGateTest(unittest.TestCase):
    def test_source_switch_discards_previous_command(self):
        gate = CommandGate()
        gate.accept(True, (0.3, 0., 0.), 1.)
        self.assertEqual(gate.output(1.1), (0., 0., 0.))
        gate.accept(False, (0.2, 0., 0.), 1.)
        self.assertEqual(gate.output(1.1)[0], 0.2)
        gate.select(True)
        self.assertEqual(gate.output(1.1), (0., 0., 0.))
        gate.accept(True, (0.3, 0., 0.), 2.)
        gate.accept(False, (0.4, 0., 0.), 2.1)
        self.assertEqual(gate.output(2.1)[0], 0.3)
        self.assertEqual(gate.output(2.31), (0., 0., 0.))

    def test_limits_and_invalid_commands(self):
        gate = CommandGate()
        gate.accept(False, (5., -5., 5.), 1.)
        self.assertEqual(gate.output(1.1), (0.5, -0.3, 0.8))
        gate.accept(False, (float('nan'), 0., 0.), 1.1)
        self.assertEqual(gate.output(1.1), (0., 0., 0.))
