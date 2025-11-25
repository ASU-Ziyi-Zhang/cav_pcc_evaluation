
#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

from math import fmod, ceil, sin, cos, nan, atan2, pi
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import warnings
import os

from src.settings import *

FORCE_MAP_REDRAW = False # Always redraw the s2x approximator
IS_ROTATING_COORDS = False # For debugging

class RoadFrame:
    """
        Performs coordinate transformation of road waypoints in (x,y) to frenet frame (s,l) and the inverse
        
        Attributes:
            x_road (np.array)
            y_road (np.array)
            s_length (float)

        Methods:
            distance(ego_xy, target_xy) : calculates the longitudinal gap between two points in frenet frame
            compute_s(x, y) : calculates s given x, y
            xy2sl(x, y, vx, vy, a) : calculates s, l in frenet frame
            sl2xy(s, v, a, l, ldot) : calculates x, y in cartesian frame
    """
    
    def _calculate_angles(self):
        angles = np.full_like(self.x_road, np.nan)

        road_dir = -1 if self.is_road_reversed else 1

        for i in range(1, len(angles)):
            x0 = self.x_road[i-1] 
            y0 = self.y_road[i-1]
            x1 = self.x_road[i]
            y1 = self.y_road[i]
        
            angle = atan2(road_dir*(y1 - y0), road_dir*(x1 - x0))
            angles[i] = angle

        self._angles_road = angles

    @staticmethod
    def get_sl(waypoints):
        return waypoints[:,2], waypoints[:,3]
    
    @staticmethod
    def get_xy(waypoints):
        return waypoints[:,0], waypoints[:,1]
    
    @staticmethod
    def remove_range(sh_waypoints, s_start, s_end):
        '''
            Remove waypoints outside of the range s_start and s_end

            returns sh_waypoints_mod : [x,y,s,l]
        '''
        # Check input arguments
        assert s_start < s_end, 's_start should be smaller than s_end'

        # Discard points outside s_start and s_end
        s_inds = np.logical_or(sh_waypoints[:,2] > s_end, sh_waypoints[:,2] < s_start)
        sh_waypoints[s_inds, :] = nan
        
        # Return
        return sh_waypoints

    @staticmethod
    def find_values_in_ranges(arr, ranges):
        mask = np.zeros_like(arr, dtype=bool)
        for min_val, max_val in ranges:
            mask |= (arr >= min_val) & (arr <= max_val)

        return arr[mask]
    
    @staticmethod
    def find_ranges(arr, ranges):
        mask = np.zeros_like(arr, dtype=bool)
        for min_val, max_val in ranges:
            mask |= (arr >= min_val) & (arr <= max_val)
        return mask

    def getMergeMarkers(self, sh_waypoints, s_start, s_end, l_start, l_end):
        ''' Remove lane line or shoulder waypoints to taper off a shoulder for a closing lane '''
        # TODO
        warnings.warn('not yet implemented merging shoulder lines')

        return sh_waypoints

    def getLaneMarkers(self, sh_waypoints, dividerLen=4, dividerGap=14):
        '''
            Remove lane line waypoints outside of the lane dividers

            returns: 
                x
                y

            Inputs:
                sh_waypoints # lane line waypoints [x, y, s, l]
                dividerLen = 4. # [m] Length of the lane divider marking 
                dividerGap = 14. # [m] Length in between the end points of lane divider markings
        '''
        
        s0, s1 = 0, self.s_length
        numDivMarkings = ceil(( s1-s0 ) / (dividerLen+dividerGap) )
        
        ranges = []
        for j in range(0, numDivMarkings): # Each lane marker
            s_start = j*(dividerLen+dividerGap) # Start s of a lane marker
            s_end = (j+1)*dividerLen + j*dividerGap # End s of a lane marker

            ranges.append((s_start, s_end))
            
        mask = self.find_ranges(sh_waypoints[:,2], ranges) # Find indices that correspond to where a marker is
        sh_waypoints[~mask, :] = nan # No value for the road line when not on a lane marker

        return sh_waypoints

    def getCenterLine(rf):
        '''
            Given a RoadFrame object, compute the s, l, and gamma arrays corresponding to the centerline from the waypoints

            returns s, l, gamma
        '''
        # Initialize data
        s_arr = np.zeros_like( rf.x_road )
        l_arr = np.zeros_like( rf.x_road )
        t_arr = np.zeros_like( rf.x_road ) # Gamma

        # Loop through coordinate transform model
        for i in range(0, len(s_arr)):
            x = rf.x_road[i]
            y = rf.y_road[i]

            s, _, _, l, _, t = rf.xy2sl(x, y, 0., 0., 0.)

            s_arr[i] = s
            l_arr[i] = l
            t_arr[i] = t
        
        # Return
        return s_arr, l_arr, t_arr

    def getShoulderLine(rf, n_lane = 1, type='right'):
        '''Given a RoadFrame object, returns the shoulder line to the ['left', 'right'] of a given lane
        
        Returns the [x, y, s, l] waypoints of a shoulder line'''
        
        # Check input
        assert type=='right' or type=='left', 'Unexpected shoulder type.'
        
        # Settings
        step_size = 2. # delta s
        s_start = -10 # Beginning s position

        s_end = rf.s_length + 10 # Ending s position

        # Initialize data
        n_rows = int(ceil( (s_end-s_start) / step_size ))
        sh_waypoints = np.empty(( (n_rows, 4) )) # Shoulder x-y waypoints

        # Loop through coordinate transform model    
        i = 0
        for s in np.arange(s_start, s_end, step_size):
            # Get shoulder
            l = 0.5 if type=='right' else n_lane + 0.5

            x, y, _, _, _, _ = rf.sl2xy(s, 0., 0., l, 0.)

            sh_waypoints[i, 0] = x
            sh_waypoints[i, 1] = y
            sh_waypoints[i, 2] = s
            sh_waypoints[i, 3] = l

            # Iterate
            i+=1

        return sh_waypoints # [x, y, s, l] array

    def getLaneLine(rf, n_lane = 1):
        '''Given a RoadFrame object and lane number, compute the x, y, s, l waypoints of a lane marker to the left'''

        # Check input
        assert n_lane>=0, 'Unexpected lane type.'
        
        # Settings
        sh_waypoints = rf.getShoulderLine(n_lane=n_lane, type='left')

        return sh_waypoints # [x, y, s, l] array
    
    def getStartingCoords(rf):
        '''Given a roadframe object, return the x,y starting coordinates'''

        if rf.is_road_reversed:
            return rf.x_road[-1], rf.y_road[-1]
        else:
            return rf.x_road[0], rf.y_road[0]
        
    def getEndingCoords(rf):
        '''Given a roadframe object, return the x,y starting coordinates'''

        if rf.is_road_reversed:
            return rf.x_road[0], rf.y_road[0]
        else:
            return rf.x_road[-1], rf.y_road[-1]
        
    @staticmethod
    def getStraightRoadPos(rf, x, y, v, theta):
        ''' Function to calculate s, l, and ldot for the ego assuming a straight road
        
        x0 + s cos(gamma) - l sin(gamma) = x
        
        y0 + s sin(gamma) + l cos(gamma) = y
        
        z0 + n R(gamma) = z

        gamma = atan2(y1-y0, x1-x0) # Straight road orientation angle
        '''

        # Parameters
        lw_inv = LANEWIDTH_INV

        gamma = rf.gamma
        x0, y0 = rf.getStartingCoords()
        x1, y1 = rf.getEndingCoords() #  For checking below

        # Error Check
        assert rf.is_road_straight, 'Detected Roadframe object created without a straight road is using straight-road-only methods.'
        assert abs(gamma-atan2(y1-y0, x1-x0)) < 1e-6, 'Straight road orientation angle from Roadframe object disagrees with starting and ending coordinates?'

        ### Calculate s, l
        t = gamma

        b = np.array([
            x-x0, 
            y-y0
        ])

        A = np.array([
            [cos(t), -sin(t)],
            [sin(t), cos(t)]
        ])

        # Solve
        z = np.linalg.solve(A, b)
        
        s = z[0]
        l = z[1]
        ldot = v*sin(theta-gamma)
        
        # Normalize such that l = 1.0 is the centerline of first lane, l = 2.0 is centerline of second lane, l = 3.0 is ...
        l = l*lw_inv + 1
        ldot = ldot*lw_inv

        return s, l, ldot

    @staticmethod
    def check_trend(arr):
        increasing = all(arr[i] < arr[i + 1] for i in range(len(arr) - 1))
        decreasing = all(arr[i] > arr[i + 1] for i in range(len(arr) - 1))
        
        if increasing:
            return "Increasing"
        elif decreasing:
            return "Decreasing"
        else:
            return "Neither"

    def __init__(self, road_waypoints_csv: str = 'CMI_right_lane_glob.csv', 
                 road_waypoints_xy: np.array = None,
                 n_lanes = 1):
        
        ### Check input arguments
        if road_waypoints_xy is not None:
            self.waypoints = road_waypoints_xy
            name = str( road_waypoints_xy[:,0] )

            self.is_road_straight = True

        elif road_waypoints_csv != '':
            self.waypoints = np.loadtxt(road_waypoints_csv, delimiter=',')
            name = road_waypoints_csv.split('.')[0]

            self.is_road_straight = False

        else:
            raise ValueError('RoadFrame expected waypoints as a constructor input argument')

        assert n_lanes > 0, 'Expected non-zero number of lanes. Lane 1 is the rightmost lane defined by the waypoints.'

        # Read waypoints
        self.x_road = self.waypoints[:, 0]
        self.y_road = self.waypoints[:, 1]

        self.num_waypoints = len(self.x_road)

        # Assign number of supported lanes
        self.n_lanes = n_lanes

        ### ROTATING COORDINATES debugging
        if IS_ROTATING_COORDS:
            warnings.warn("Rotating waypoints for testing purposes", UserWarning)
            # input('pausing for user confirm..')

            self.x_road = -self.waypoints[:, 1]
            self.y_road = self.waypoints[:, 0]

        ### Check direction of waypoints and fit spline
        trend = self.check_trend(self.x_road)
        if trend=='Increasing':
            self.is_road_reversed = False
        
        elif trend=='Decreasing':
            print('Direction of travel from right to left detected. Reversing coordinates.')

            self.is_road_reversed = True

            self.x_road = np.flip(self.x_road) # Spline function needs x to be strictly increasing
            self.y_road = np.flip(self.y_road)

        else:
            raise ValueError('RoadFrame x-coordinates are looped? Both increasing and decreasing values detected.')

        self.cspline = sp.interpolate.CubicSpline(self.x_road, self.y_road)

        # Projection values initialize
        self.x_cs = 0. # s to xy projection
        self.y_cs = 0.
        self.x_proj = 0. # xy to s projection
        self.y_proj = 0.

        ## Calculate road tangent       
        self.slope_func = self.cspline.derivative()

        ## Calculate road lookup table
        self._s2x_approximator(name=name)

        ## Once lookup table is constructed -- calculate road waypoints s, l, gamma
        self.s_road, self.l_road, self.gamma_road = self.getCenterLine()

        # Calculate right and left bounding shoulders
        self.right_shoulder = self.getShoulderLine(type='right', n_lane=1)
        self.left_shoulder = self.getShoulderLine(type='left', n_lane=self.n_lanes)
    
        # Calculate the lanes with the non lane marker waypoints nan'd
        self.lanes = {}
        for i in range(1, self.n_lanes): # Lanes start from 1
            self.lanes[i] = self.getLaneMarkers( self.getLaneLine(n_lane = i) )

    def distance(self, ego_x, target_x, ego_y, target_y):
        '''function to find the along-the-road frenet gap between the ego x-y points and the target x-y points'''
        s_ego = self.compute_s(ego_x, ego_y) # get ego s

        s_target = self.compute_s(target_x, target_y) # get target s

        return s_target - s_ego

    # xy to sl
    def _f_dis(self, point, x_sp):
        """ function to find distance between a road point and a x-parametric point on c-spline """
        y_sp = self.cspline(x_sp)

        return np.linalg.norm(point - np.array([x_sp, y_sp]))
    
    # sl to xy
    def _s_dis(self, x_cs, s):
        """ function to compute distance between s on spline and parametrized point cspline(x_cs) """
        sdis = 0.0
        i = 0
        
        try:
            while self.x_road[i+1] <= x_cs:
                # straight = np.linalg.norm(np.array([self.x_road[0], self.cspline(self.x_road[0])]) - np.array([self.x_road[i], self.cspline(self.x_road[i])]))
                sdis += np.linalg.norm(np.array([self.x_road[i], self.cspline(self.x_road[i])]) - np.array([self.x_road[i+1], self.cspline(self.x_road[i+1])]))
                i += 1
            
            sdis += np.linalg.norm(np.array([self.x_road[i], self.cspline(self.x_road[i])]) - np.array([x_cs, self.cspline(x_cs)]))
            
            return np.abs(sdis - s)
        
        except:
            raise Exception("Out of bounds")
        
    def _arc_len(self, x_cs):
        f = lambda x: np.sqrt(1 + (self.slope_func(x)) ** 2)
        s_len = sp.integrate.quad(f, self.x_road[0], x_cs, epsabs=1.49e-06, epsrel=1.49e-06, limit=100) # default epsabs=1.49e-08, epsrel=1.49e-08, limit=50
        
        return s_len[0]
    
    def _s_eval(self, s, x_cs):
        return np.linalg.norm(s - self._arc_len(x_cs))
    
    def _way_point_res(self):
        distances = []
        for i in range(len(self.waypoints)-1):
            pa = self.waypoints[i, 0:2]
            pb = self.waypoints[i+1, 0:2]

            distances.append(np.linalg.norm(pa - pb))

        self.s_length = np.sum(distances)
        
    def compute_s(self, x, y):
        """ computes s til projected point on cspline & index of closest road waypoint (for other vector calcs) """
        self.x_proj, self.y_proj = self._project_point(x, y)

        s = 0.0
        i = 0
        try:
            while self.x_road[i+1] <= self.x_proj:
                s += np.linalg.norm(np.array([self.x_road[i], self.cspline(self.x_road[i])]) - np.array([self.x_road[i+1], self.cspline(self.x_road[i+1])]))
                i += 1

            s += np.linalg.norm(np.array([self.x_road[i], self.cspline(self.x_road[i])]) - np.array([self.x_proj, self.cspline(self.x_proj)]))

        except:
            raise ValueError('Out of bounds at i={:d}'.format(i))
        
        # If reversed coordinates then the start is actually the end of the waypoints
        if self.is_road_reversed:
            s_end = self.s_length
            s = s_end-s

        return s

    def _project_point(self, x, y):
        vehicle_point = np.array([x, y])

        self.opt = sp.optimize.minimize_scalar(lambda x_sp: self._f_dis(vehicle_point, x_sp), bounds=(self.x_road[0], self.x_road[-1]))
        x_proj = self.opt.x
        y_proj = self.cspline(x_proj)

        return x_proj, y_proj
        
    def _s2x_bounds(self, s):
        # Setting
        threshold = 1.00

        # Initialize
        i = 0
        p1 = np.array([self.x_road[i], self.y_road[i]])
        p2 = np.array([self.x_road[i+1], self.y_road[i+1]])

        dis = 0.0

        while np.linalg.norm(s - dis) > threshold and i < self.num_waypoints:
            p1 = np.array([self.x_road[i], self.y_road[i]])
            p2 = np.array([self.x_road[i+1], self.y_road[i+1]])
            dis += np.linalg.norm(p2 - p1)
        
            i += 1
        
        return p1[0], p2[0]
    
    def _s2x_approximator(self, name='map', approximation_res=5e-1, n_lane=1):
        """Function approximator for generating a lookup table. Checks if s2x_lookup.txt exists and loads it, otherwise calculates table"""
        
        filename = 's2x_lookup_' + name +  '_lane' + str(n_lane) + '.txt'

        # Check if waypoint lookup table has been generated
        if os.path.exists(filename) and not FORCE_MAP_REDRAW:
            print('Loading s to x map {:s}'.format(filename))
            self.sx_map = np.loadtxt(filename, delimiter=',')
            self.s_length = self.sx_map[-1, 0]

        # Otherwise generate lookup table
        else:
            print('Did not find map {:s} - generating a new one.'.format(filename))
            self._way_point_res()

            s_series = np.arange(0.0, self.s_length, step=approximation_res)
            s_x = np.zeros((len(s_series), 2))
            
            for i in range(len(s_series)):    
                if fmod(i, 100) == 0:
                    print( "Generating s to x: {:d}/{:d}".format(i, len(s_series)) )
            
                self.opt = sp.optimize.minimize_scalar(lambda x_cs: self._s_eval(s_series[i], x_cs), bounds=(self.x_road[0], self.x_road[-1]))
                
                s_x[i, 0] = s_series[i]
                s_x[i, 1] = self.opt.x

            self.sx_map = s_x
            np.savetxt(filename, s_x, fmt="%f", delimiter=",")

    def _s2x_lookup(self, s):
        if self.is_road_reversed:
            s_map = self.s_length - s
        else:
            s_map = s
    
        return np.interp(s_map, self.sx_map[:, 0], self.sx_map[:, 1]) # Assumes x is monotic and y is a function - same as cspline assumptions

    def xy2sl(self, x, y, vx, vy, a):
        ''' Compute the frenet frame coordinates given cartesian, x, y, vx, vy, a
            
            returns s, s_dot, s_ddot, l, l_dot, gamma
            
            l=1 on the rightmost lane, l=2 on second lane, ...
        '''
        ### Compute s
        s = self.compute_s(x, y)

        ### Compute s dot, l, l dot
        # Road direction
        road_dir = -1 if self.is_road_reversed else 1
        gamma = np.arctan2( road_dir*self.slope_func(self.x_proj), road_dir ) # requires compute_s to be run first for x_proj

        s_dir = np.array([np.cos(gamma), np.sin(gamma)])

        vehicle_point = np.array([x, y])
        diff = vehicle_point - np.array([self.x_proj, self.y_proj]) # requires compute_s to be run first
        
        p_norm = np.linalg.norm(( diff ))

        l_dir = diff / p_norm if p_norm > 0 else np.array([0., 0.])
        
        d = self.opt.fun * LANEWIDTH_INV
        sxl = np.cross(s_dir, l_dir) # requires compute s dot to be run first

        if sxl >= 0.0:
            l = d
        else:
            l = -d

        # Velocity
        v = np.array([vx, vy]) # \dot{y} = v_x (-\sin(\psi)) + v_y (\cos(\psi))
        v_norm = np.linalg.norm(v)
        v_hat = np.array([vx, vy]) / v_norm if v_norm > 0 else np.array([0., 0.])

        s_dot = np.dot(v, s_dir)

        # Lateral Velocity # Cross product with road direction - left is growing positive
        l_dot = np.cross(v, -s_dir)

        # Forward acceleration
        a_net = a * v_hat
        s_ddot = np.dot(a_net, s_dir)

        # Lateral acceleration # Cross product with road direction - left is growing positive
        l_ddot = np.cross(a_net, -s_dir) ## 

        ### Normalize such that l = 1.0 is the centerline of first lane, l = 2.0 is centerline of second lane, l = 3.0 is ...
        # lanes = [m]/lane_w
        l += 1
        l_dot *= LANEWIDTH_INV
        l_ddot *= LANEWIDTH_INV

        return s, s_dot, s_ddot, l, l_dot, gamma

    def sl2xy(self, s, v, a, l, ldot, method='lookup'):
        ''' Compute the cartesian frame coordinates given frenet s, v, a, l, ldot

            returns x, y, vx, vy, a, gamma
        '''
        # Check input args
        assert method == 'precise' or method == 'lookup' or method == 'crude', 'Unhandled method type.'

        # Convert where rightmost centerline l=1 to l=0 for road frame
        l -= 1

        # Calculate x
        if method == 'precise':
            self.opt = sp.optimize.minimize_scalar(
                lambda x_cs: self._s_dis(x_cs, s), bounds=(self.x_road[0], self.x_road[-1])
            )
            
            x_cs = self.opt.x

        elif method == 'crude':
            x_l, x_u = self._s2x_bounds(s)
            
            x_cs = x_l

        elif method == 'lookup':
            x_cs = self._s2x_lookup(s)
        
        # Calculate coordinates
        y_cs = self.cspline(x_cs)

        road_dir = -1 if self.is_road_reversed else 1
        g_ = np.arctan2( road_dir*self.slope_func(x_cs), road_dir )

        si = np.cos( g_ )
        sj = np.sin( g_ )

        s_hat = np.array([si, sj, 0])
        k_hat = np.array([0, 0, 1])
        l_hat = (l / abs(l)) * np.cross(k_hat, s_hat) if abs(l) > 1e-6 else np.array([0, 0, 0])

        x_ = x_cs + (abs(l) * LANEWIDTH) * np.dot(l_hat, np.array([1, 0, 0]))
        y_ = y_cs + (abs(l) * LANEWIDTH) * np.dot(l_hat, np.array([0, 1, 0]))

        v_net = v * s_hat + ldot * l_hat
        vx_ = np.dot(v_net, np.array([1, 0, 0]))
        vy_ = np.dot(v_net, np.array([0, 1, 0]))

        a_ = a # MPC assumes no lateral acc

        # Return
        self.x_cs = x_cs
        self.y_cs = y_cs

        return x_, y_, vx_, vy_, a_, g_

if __name__ == '__main__':
    N_LANES = 2 # Number of lanes that the roadframe will calculate lines for - number of lanes to plot below
    IS_USING_STRAIGHT_ROAD = False # For debugging TODO
    DIR = 1

    ### Test coordinate transformation from csv
    if not IS_USING_STRAIGHT_ROAD:
        if DIR == 0:
            road_waypoints_csv = "CMI_right_lane_glob.csv"

        elif DIR == 1:
            road_waypoints_csv = "CMI_right_lane_reversed.csv"
        
        # Make the road frame using csv
        rf = RoadFrame(road_waypoints_csv=road_waypoints_csv, 
                       n_lanes = N_LANES)

        # Debugging
        expected_gamma = nan

    ### Test coordinate transformation for straight road at track
    # ITIC is set as a straight road but 2 possible directions:
    else:
        if DIR == 0:
            x0, y0 = 308.313, -156.179 # From 3-lane ITIC map LONG
            x1, y1 = 587.9804, -1531.023 # From 3-lane ITIC map LONG

        elif DIR == 1:
            x0, y0 = 595.2319, -1529.5 # Start point - From 3-lane ITIC map LONG
            x1, y1 = 315.566, -154.709 # End point - From 3-lane ITIC map LONG

        # Make the road frame using start and end points of straight segment
        rf = RoadFrame(road_waypoints_xy=np.array( ([x0, y0], [x1, y1]) ), 
                    n_lanes=N_LANES)
        
        # Debugging
        expected_gamma = atan2(y1 - y0, x1 - x0)

    # Example current vehicle sensor info
    ## Settings
    is_plotting_c2f = True
    is_plotting_f2c = True
    is_plotting_tan = True
    is_plotting_xs = True
    is_plotting_f2clane = True
    is_plotting_f2clane_left = False # From csv
    is_plotting_gap = False

    ##
    if is_plotting_c2f:
        x = 290.4969499222862 # Example GPS measurement        
        y = -222.0103088757058
        vx = 5.0
        vy = -2.0
        a = 0.5

        ### Rotated test coordinates
        if IS_ROTATING_COORDS:
            x, y = -y, x

        s, v, a_s, l, ldot, gamma = rf.xy2sl(x, y, vx, vy, a)

        print(f"x: {x}, y: {y}, vx: {vx}, vy: {vy}, a: {a}")
        print(f"s: {s}, v: {v}, a: {a_s}, l: {l}, ldot: {ldot}")
    
        plt.figure()
        plt.plot(rf.x_road, rf.cspline(rf.x_road), label="Spline")
        plt.plot(x, y, 'ro', label="vehicle point")
        plt.plot(rf.x_proj, rf.y_proj, 'go', label="projection point")
        ax = plt.gca()  # Get the current axes
        ax.set_aspect('equal', adjustable='box')  # Equal aspect ratio

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend()
        plt.title( 'Cartesian to Frenet: gamma={:0.2f}, expected={:0.2f}'.format(gamma, expected_gamma) )

    ##
    if is_plotting_f2c:
        s, v, a_s, l, ldot = 100., 10., 0., 1.1, 0.

        x, y, vx, vy, a, tangent = rf.sl2xy(s, v, a_s, l, ldot)

        expected_gamma = np.interp(x, rf.x_road, rf.gamma_road)

        plt.figure()
        plt.plot(rf.x_road, rf.cspline(rf.x_road), label="Spline")
        plt.plot(x, y, 'ro', label="vehicle point")
        plt.plot(rf.x_cs, rf.y_cs, 'go', label="projection point")
        ax = plt.gca()  # Get the current axes
        ax.set_aspect('equal', adjustable='box')  # Equal aspect ratio

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend()
        plt.title( 'Frenet to Cartesian: gamma={:0.2f}, expected={:0.2f}'.format(tangent, expected_gamma) )

    ##
    if is_plotting_tan:
        # Check angles numerically
        rf._calculate_angles()

        # Plot
        fig, ax = plt.subplots(2,1, sharex=True)
        
        ax[0].plot(rf.x_road, rf.cspline(rf.x_road), label="Centerline")
        ax[0].set_ylabel("y [m]")
        # ax[0].set_aspect('equal', adjustable='box') # Equal aspect ratio

        ax[1].plot(rf.x_road, rf.gamma_road, label='Tangent (1st Derivative)', linestyle='--')
        ax[1].plot(rf.x_road, rf._angles_road, label='Numerical diff', linestyle='--', color='r')
        ax[1].set_xlabel("x [m]")
        ax[1].set_ylabel("\gamma [rad]")
        ax[1].legend()
        
        plt.tight_layout()

    if is_plotting_xs:
        fig, ax = plt.subplots(1,1, sharex=True)
        
        ax.plot(rf.x_road, rf.s_road, label="Centerline")
        ax.set_ylabel("s [m]")
        ax.set_xlabel('x [m]')
        ax.set_aspect('equal', adjustable='box') # Equal aspect ratio
        
        plt.tight_layout()

    ##
    if is_plotting_f2clane:
        fig, ax = plt.subplots(2,1)

        rx, ry = rf.get_xy( rf.right_shoulder )
        lx, ly = rf.get_xy( rf.left_shoulder )

        # Plot Shoulders xy
        ax[0].plot(rf.x_road, rf.y_road, color='b', linewidth=0.5, label='centerline')
        
        ax[0].plot(rx, ry, color='k', linewidth=1.5, label='right')
        ax[0].plot(lx, ly, color='k', alpha = 0.6, linewidth=1.5, label='left')

        # Plot lanes xy
        for i in range(1, N_LANES):
            if i in rf.lanes:
                lane_x, lane_y = rf.get_xy( rf.lanes[i] )
                ax[0].plot(lane_x, lane_y, color='k', linewidth=0.9, label='lane')
        
        ax[0].set_ylabel('y')
        ax[0].set_xlabel('x')
        ax[0].set_aspect('equal', adjustable='box')
        ax[0].set_title('sl2xy with road shoulders')
        
        # Plot shoulders sl
        rs, rl = rf.get_sl( rf.right_shoulder )
        ls, ll = rf.get_sl( rf.left_shoulder )

        ax[1].plot(rf.s_road, rf.l_road, color='b', linewidth=0.5, label='centerline')
        
        ax[1].plot(rs, rl, color='k', linewidth=1.5, label='right')
        ax[1].plot(ls, ll, color='k', alpha = 0.6, linewidth=1.5, label='left')

        # Plot lanes sl
        for i in range(1, N_LANES):
            if i in rf.lanes:
                lane_s, lane_l = rf.get_sl( rf.lanes[i] )
                ax[1].plot(lane_s, lane_l, color='k', linewidth=0.9, label='lane')
        
        ax[1].set_ylabel('l')
        ax[1].set_xlabel('s')
        ax[1].set_ylim([0, N_LANES+1])
        ax[1].set_title('xy2sl with road shoulders')
        
        plt.legend()

    if is_plotting_f2clane_left:
        test_waypoints_csv = "CMI_left_lane_glob.csv"
        test_waypoints = np.loadtxt(test_waypoints_csv, delimiter=',')
        
        test_x = test_waypoints[:,0]
        test_y = test_waypoints[:,1]

        ### Rotated test coordinates
        if IS_ROTATING_COORDS:
            test_x = -test_waypoints[:,1]
            test_y = test_waypoints[:,0]
        
        ##
        fig, ax = plt.subplots(2,1)

        # Plot
        ax[0].plot(rf.x_road, rf.y_road, color='b', linewidth=0.5, label='centerline')
        ax[0].plot(test_x, test_y, color='r', linewidth=0.5, label='centerline')

        rx, ry = rf.get_xy( rf.right_shoulder )
        lx, ly = rf.get_xy( rf.left_shoulder )

        rs, rl = rf.get_sl( rf.right_shoulder )
        ls, ll = rf.get_sl( rf.left_shoulder )

        # Plot
        ax[0].plot(rf.x_road, rf.y_road, color='b', linewidth=0.5, label='centerline')
        ax[0].plot(rx, ry, color='k', linewidth=1.5, label='right')
        ax[0].plot(lx, ly, color='k', alpha = 0.6, linewidth=1.5, label='left')
        
        ax[0].set_ylabel('y')
        ax[0].set_xlabel('x')
        ax[0].set_aspect('equal', adjustable='box')
        ax[0].set_title('sl2xy with road shoulders')
        
        #
        ax[1].plot(rf.s_road, rf.l_road, color='b', linewidth=0.5, label='centerline')
        ax[1].plot(rs, ls, color='k', linewidth=1.5, label='right')
        ax[1].plot(ls, ll, color='k', linewidth=1.5, label='left')
        
        ax[1].set_ylabel('l')
        ax[1].set_xlabel('s')
        ax[1].set_ylim([0, 2.5])
        ax[1].set_title('xy2sl with road shoulders')
        
        plt.legend()
        
    # Close plots
    plt.show(block=False)
    
    input("Press Enter to continue...") # raw_input() in python2
    plt.close('all')