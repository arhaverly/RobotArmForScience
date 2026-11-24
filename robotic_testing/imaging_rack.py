"""
Leave a sample on an imaging rack, and take it back.

An imaging rack holds an electrode lying left to right, with its coated end under a camera that looks
down on it. So that the electrode never comes down through the lens, the arm never comes straight down
onto the imaging spot. It comes down beside the spot, on the metal-end side (away from the camera),
lowers the electrode to just above the rack, slides it in under the camera and puts it down there.
Taking it back is the same in reverse.

The moves between mid_station and the imaging rack cross high above the bench (``safe_z``), turning the
gripper a quarter turn on the way. Everything else -- picking up, the flask, mid_station, putting back --
uses the arm's own routines, exactly as the immersion sweep in AIsorb_arm.ipynb does.

The spot is taught with imaging_right.ipynb (its teach cell), holding a real sample.
"""
import time


class ImagingRack:
    def __init__(self, arm, spot, coated_end='right', safe_z=500.0, hover_mm=100.0, slide_mm=70.0,
                 slide_height_mm=25.0, travel_speed=100.0, slide_speed=30.0, place_speed=10.0,
                 confirm_before_placing=True):
        """
        :param arm: the connected arm (rt.arm)
        :param spot: [x, y, z] where the gripper lets go of the electrode on the imaging rack, in mm
        :param coated_end: 'right' or 'left' -- which side of the rack the coated end (and the camera) is on
        :param safe_z: travel height between mid_station and the imaging rack
        :param hover_mm: how high above the spot the arm stops to ask before putting the sample down
        :param slide_mm: how far towards the metal end, away from the camera, the arm comes down
        :param slide_height_mm: height above the spot at which the electrode slides in and out
        :param travel_speed: mm/s from mid_station up, across, and down beside the imaging rack
        :param slide_speed: mm/s going low over the rack and sliding in or out under the camera
        :param place_speed: mm/s for the last part down into the slot, and back up from it
        :param confirm_before_placing: stop above the imaging rack and wait for a yes
        """
        if coated_end not in ('left', 'right'):
            raise ValueError("coated_end must be 'left' or 'right'")
        self.arm = arm
        self.cfg = arm.config
        self.x, self.y, self.z = (float(v) for v in spot)
        self.coated_end = coated_end
        # gripper pointing down, electrode left to right: yaw 180 puts the coated end on the right, 0 on the left
        self.rpy = [180.0, 0.0, 180.0 if coated_end == 'right' else 0.0]
        self.safe_z = float(safe_z)
        self.hover_mm = float(hover_mm)
        self.side_x = self.x - slide_mm if coated_end == 'right' else self.x + slide_mm
        self.slide_z = self.z + slide_height_mm
        self.travel_speed, self.slide_speed, self.place_speed = travel_speed, slide_speed, place_speed
        for speed in (travel_speed, slide_speed, place_speed):
            self.at_speed(speed)            # raises now if a speed is out of range, before anything moves
        self.confirm_before_placing = confirm_before_placing
        self.mid = list(self.cfg.pos_dict['mid_station'])

    # ------------------------------------------------------------------ helpers

    def at_speed(self, mm_s):
        """The arm's default motion settings with the speed swapped to mm_s."""
        if not 0 < mm_s <= 100:
            raise ValueError(f'Speed {mm_s} mm/s must be between 0 and 100')
        return {**self.cfg.pos_settings_dict['default'], 'speed': mm_s}

    @staticmethod
    def turned_from(rpy_a, rpy_b):
        """Largest angle (degrees) between two orientations given as roll, pitch, yaw."""
        return max(abs(((a - b) + 180) % 360 - 180) for a, b in zip(rpy_a, rpy_b))

    def go(self, steps):
        for name, pose, speed in steps:
            print('   ->', name)
            self.arm.move_to_pos(pose, **self.at_speed(speed))

    def describe(self):
        return (f'Imaging rack, coated end on the {self.coated_end}: lets go at '
                f'x = {self.x:.1f}, y = {self.y:.1f}, z = {self.z:.1f}; '
                f'{self.travel_speed} mm/s high above the bench, {self.slide_speed} mm/s sliding in, '
                f'{self.place_speed} mm/s into the slot; '
                f'{"stops to ask before putting a sample down" if self.confirm_before_placing else "does not stop to ask"}.')

    # ------------------------------------------------------------------ the moves

    def _to_rack(self):
        """mid_station -> high across, turning -> down beside the imaging spot (where the arm stops to ask)."""
        mid, s = self.mid, self.travel_speed
        return [
            ('straight up from mid_station', [mid[0], mid[1], self.safe_z, *mid[3:]], s),
            ('across, turning a quarter turn', [self.side_x, self.y, self.safe_z, *self.rpy], s),
            ('down, beside the imaging spot', [self.side_x, self.y, self.z + self.hover_mm, *self.rpy], s),
        ]

    def _into_slot(self):
        """Beside the imaging spot -> low over the rack -> slide in under the camera -> down into the slot."""
        return [
            ('down, low over the rack', [self.side_x, self.y, self.slide_z, *self.rpy], self.slide_speed),
            ('slide in under the camera, to the imaging spot', [self.x, self.y, self.slide_z, *self.rpy], self.slide_speed),
            ('down into the slot', [self.x, self.y, self.z, *self.rpy], self.place_speed),
        ]

    def _out_to_mid(self):
        """In the slot -> up a little -> slide out, away from the camera -> high across, turning back -> mid_station."""
        mid, s = self.mid, self.travel_speed
        return [
            ('up a little', [self.x, self.y, self.slide_z, *self.rpy], self.place_speed),
            ('slide out, away from the camera', [self.side_x, self.y, self.slide_z, *self.rpy], self.slide_speed),
            ('straight up', [self.side_x, self.y, self.safe_z, *self.rpy], s),
            ('across, turning back', [mid[0], mid[1], self.safe_z, *mid[3:]], s),
            ('down to mid_station', mid, s),
        ]

    def _from_stop_to_mid(self):
        """From where the arm stops to ask -> straight up -> high across, turning back -> mid_station."""
        mid = self.mid
        return [
            ('straight up', [self.side_x, self.y, self.safe_z, *self.rpy], 30),
            ('across, turning back', [mid[0], mid[1], self.safe_z, *mid[3:]], 30),
            ('down to mid_station', mid, 30),
        ]

    def _start(self, what):
        """
        Print what is about to happen, wait for a yes, then bring the arm to mid_station high above the
        bench if it is not there already. Stops the cell (nothing moved) unless the gripper points straight
        down at least 5 cm above the racks.
        """
        here = self.arm.get_cartesian_pos()
        mid = self.mid
        if min(self.turned_from(here[3:], mid[3:]), self.turned_from(here[3:], self.rpy)) > 5 or here[2] < 240:
            raise SystemExit('The gripper must point straight down and be at least 5 cm above the racks '
                             '(z >= 240) before this cell. The arm did not move.')
        at_mid = (max(abs(a - b) for a, b in zip(here[:3], mid[:3])) <= 5
                  and self.turned_from(here[3:], mid[3:]) <= 5)
        route = [] if at_mid else [
            ('straight up from where the arm is', [here[0], here[1], self.safe_z, *here[3:]], 30),
            ('across to above mid_station', [mid[0], mid[1], self.safe_z, *mid[3:]], 30),
            ('down to mid_station', mid, 30),
        ]
        print(what)
        for name, pose, speed in route:
            print(f'  first: {name:40s} -> {[round(v, 1) for v in pose]} at {speed} mm/s')
        print('Check: red wire has slack, hand on the e-stop, and the camera lens at least 5 cm above the rack.')
        if input('Type yes to start: ').strip().lower() != 'yes':
            raise SystemExit('Cancelled, the arm did not move.')
        self.arm.assert_ready()
        if route:
            print('[0] to mid_station, high above the bench')
            self.go(route)

    def _leave(self, sample):
        """From mid_station, holding the sample: put it on the imaging rack, let go, back to mid_station."""
        print(f'[*] to the imaging rack at {self.travel_speed} mm/s')
        self.go(self._to_rack())
        if self.confirm_before_placing:
            while True:
                answer = input(f'Gripper above the imaging rack, coated end on the {self.coated_end}, nothing in the '
                               'way of sliding in under the camera? Type yes to put the sample down, '
                               'or back to take it back to its own slot: ').strip().lower()
                if answer in ('yes', 'back'):
                    break
                print('Type yes or back.')
            if answer == 'back':
                print(f'[*] taking sample {sample} back to its own slot')
                self.go(self._from_stop_to_mid())
                self.arm.put_sample_back(sample)
                raise SystemExit(f'Sample {sample} is back in its own slot; nothing was left on the imaging rack.')
        self.go(self._into_slot())
        self.arm.open_gripper()
        print(f'[*] sample {sample} left on the imaging rack; back to mid_station')
        self.go(self._out_to_mid())

    # ------------------------------------------------------------------ what the cells call

    def dip_and_leave(self, sample, dip_speed, hold_s, lift_speed):
        """
        Pick up the sample, dip, hold and lift it exactly as the immersion sweep does (through
        mid_station), then leave it on the imaging rack instead of its own slot.
        """
        cfg, arm = self.cfg, self.arm
        for speed in (dip_speed, lift_speed):
            self.at_speed(speed)
        immersed = cfg.pos_dict['flask_contact_immersed']
        above_flask = list(immersed[:2]) + [immersed[2] + cfg.hover_offset_dict['flask']] + list(immersed[3:])
        self._start(f'Sample {sample}: dip {dip_speed} mm/s, hold {hold_s} s, lift {lift_speed} mm/s, then leave it '
                    f'on the imaging rack. The imaging slot must be empty.')

        print(f'[1] pick up sample {sample}, then to the flask')
        arm.pick_up_sample(sample)
        arm.move_to_mid_station()
        arm.move_to_flask()
        print(f'[2] dip at {dip_speed} mm/s')
        arm.move_to_pos(immersed, **self.at_speed(dip_speed))
        arm.sample_in_flask = True
        if hold_s > 0:
            print(f'[3] hold {hold_s} s')
            time.sleep(hold_s)
        print(f'[4] lift at {lift_speed} mm/s, then to mid_station (at the arm\'s usual 100 mm/s)')
        arm.move_to_pos(above_flask, **self.at_speed(lift_speed))
        arm.sample_in_flask = False
        arm.move_to_mid_station()

        self._leave(sample)
        print(f'Done. Sample {sample} is on the imaging rack. Its own slot is empty.')

    def test_leave(self, sample):
        """No dipping: take the sample from its own slot straight to the imaging rack."""
        self._start(f'TEST: sample {sample} from its own slot to the imaging rack, no dipping. '
                    f'The imaging slot must be empty.')
        print(f'[1] pick up sample {sample}, then to mid_station')
        self.arm.pick_up_sample(sample)
        self.arm.move_to_mid_station()
        self._leave(sample)
        print(f'Done. Sample {sample} is on the imaging rack.')

    def take_back(self, sample):
        """Fetch the sample from the imaging rack and put it back in its own slot."""
        arm = self.arm
        self._start(f'Take sample {sample} from the imaging rack back to its own slot. Its own slot must be empty.')
        print('[1] to the imaging rack')
        self.go(self._to_rack())
        arm.open_gripper()
        self.go(self._into_slot())
        arm.close_gripper()
        opening = arm.gripper_opening   # about 9 mm when it holds an electrode, about 0 when it closes on nothing
        if opening < 3:
            print(f'The gripper closed on nothing (opening {opening:.1f} mm). Going back without the sample.')
            arm.open_gripper()
            self.go(self._out_to_mid())
            raise SystemExit('Nothing was taken. Check where the sample is on the imaging rack.')
        print('[2] got it; out from under the camera and back to mid_station')
        self.go(self._out_to_mid())
        print('[3] into its own slot')
        arm.put_sample_back(sample)
        print(f'Done. Sample {sample} is back in its own slot.')
