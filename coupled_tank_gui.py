import tkinter as tk
from tkinter import ttk
from tkinter.filedialog import asksaveasfilename
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import tkinter.messagebox as messagebox
import csv
import os

# --- LỚP BỘ ĐIỀU KHIỂN PID ---
class PIDController:
    """
    Một lớp để triển khai bộ điều khiển PID (Proportional-Integral-Derivative).
    """
    def __init__(self, Kp, Ki, Kd, set_point, output_limits=(0, 300)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.output_min, self.output_max = output_limits

        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        self._last_error = 0
        
        self.reset()

    def update(self, process_variable, dt):
        """
        Tính toán đầu ra của bộ điều khiển PID.

        Args:
            process_variable (float): Giá trị đo được hiện tại từ quá trình (ví dụ: mực nước).
            dt (float): Khoảng thời gian (delta time) kể từ lần cập nhật cuối cùng.

        Returns:
            float: Giá trị điều khiển đầu ra.
        """
        if dt <= 0:
            return self._last_output

        error = self.set_point - process_variable
        
        # Thành phần tỉ lệ (Proportional)
        self._proportional = self.Kp * error
        
        # Thành phần tích phân (Integral)
        self._integral += self.Ki * error * dt
        # Giới hạn thành phần tích phân để tránh "integral windup"
        self._integral = max(min(self._integral, self.output_max), self.output_min)

        # Thành phần đạo hàm (Derivative)
        delta_error = error - self._last_error
        self._derivative = self.Kd * (delta_error / dt) if dt > 0 else 0

        # Tính toán đầu ra tổng
        output = self._proportional + self._integral + self._derivative
        
        # Cập nhật trạng thái
        self._last_error = error
        self._last_output = max(min(output, self.output_max), self.output_min)
        
        return self._last_output

    def reset(self):
        """Đặt lại trạng thái của bộ điều khiển PID."""
        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        self._last_error = 0
        self._last_output = 0

    def set_gains(self, Kp, Ki, Kd):
        """Cập nhật các hệ số khuếch đại PID."""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def set_setpoint(self, set_point):
        """Cập nhật giá trị đặt."""
        self.set_point = set_point
        self.reset()


# --- LỚP BỘ ĐIỀU KHIỂN FUZZY PID ---
class FuzzyPIDController:
    """
    Mamdani-style Fuzzy PID Controller for coupled tank system.
    Uses fuzzy logic to dynamically tune Kp, Ki, Kd based on error and change of error.
    """
    def __init__(self, set_point, output_limits=(0, 300)):
        self.set_point = set_point
        self.output_min, self.output_max = output_limits
        self._last_error = 0
        self._last_output = 0
        self._integral = 0
        
        # Linguistic variables for error (E) and change of error (CE)
        self.linguistic_terms = ['NB', 'NM', 'NS', 'ZO', 'PS', 'PM', 'PB']  # Negative Big to Positive Big
        
        # Universe of discourse for error and change of error
        self.error_universe = np.linspace(-50, 50, 101)  # Error range: -50 to 50 cm
        self.ce_universe = np.linspace(-20, 20, 81)      # Change of error range: -20 to 20 cm/s
        
        # Universe of discourse for output gains
        self.kp_universe = np.linspace(0, 200, 101)      # Kp range: 0 to 200
        self.ki_universe = np.linspace(0, 50, 101)       # Ki range: 0 to 50
        self.kd_universe = np.linspace(0, 300, 101)      # Kd range: 0 to 300
        
        # Initialize membership functions
        self._init_membership_functions()
        
        # Initialize fuzzy rule base for Kp, Ki, Kd tuning
        self._init_rule_base()
    
    def _init_membership_functions(self):
        """Initialize triangular membership functions for all linguistic variables."""
        # Membership functions for Error (E)
        self.e_mf = {
            'NB': self._triangular_mf(self.error_universe, -50, -50, -30),
            'NM': self._triangular_mf(self.error_universe, -50, -30, -10),
            'NS': self._triangular_mf(self.error_universe, -30, -10, 0),
            'ZO': self._triangular_mf(self.error_universe, -10, 0, 10),
            'PS': self._triangular_mf(self.error_universe, 0, 10, 30),
            'PM': self._triangular_mf(self.error_universe, 10, 30, 50),
            'PB': self._triangular_mf(self.error_universe, 30, 50, 50)
        }
        
        # Membership functions for Change of Error (CE)
        self.ce_mf = {
            'NB': self._triangular_mf(self.ce_universe, -20, -20, -12),
            'NM': self._triangular_mf(self.ce_universe, -20, -12, -4),
            'NS': self._triangular_mf(self.ce_universe, -12, -4, 0),
            'ZO': self._triangular_mf(self.ce_universe, -4, 0, 4),
            'PS': self._triangular_mf(self.ce_universe, 0, 4, 12),
            'PM': self._triangular_mf(self.ce_universe, 4, 12, 20),
            'PB': self._triangular_mf(self.ce_universe, 12, 20, 20)
        }
        
        # Membership functions for Kp output
        self.kp_mf = {
            'NB': self._triangular_mf(self.kp_universe, 0, 0, 50),
            'NM': self._triangular_mf(self.kp_universe, 0, 50, 100),
            'NS': self._triangular_mf(self.kp_universe, 50, 100, 150),
            'ZO': self._triangular_mf(self.kp_universe, 100, 150, 200),
            'PS': self._triangular_mf(self.kp_universe, 150, 200, 200),
            'PM': self._triangular_mf(self.kp_universe, 200, 200, 200),
            'PB': self._triangular_mf(self.kp_universe, 200, 200, 200)
        }
        
        # Membership functions for Ki output
        self.ki_mf = {
            'NB': self._triangular_mf(self.ki_universe, 0, 0, 10),
            'NM': self._triangular_mf(self.ki_universe, 0, 10, 20),
            'NS': self._triangular_mf(self.ki_universe, 10, 20, 30),
            'ZO': self._triangular_mf(self.ki_universe, 20, 30, 40),
            'PS': self._triangular_mf(self.ki_universe, 30, 40, 50),
            'PM': self._triangular_mf(self.ki_universe, 40, 50, 50),
            'PB': self._triangular_mf(self.ki_universe, 50, 50, 50)
        }
        
        # Membership functions for Kd output
        self.kd_mf = {
            'NB': self._triangular_mf(self.kd_universe, 0, 0, 50),
            'NM': self._triangular_mf(self.kd_universe, 0, 50, 100),
            'NS': self._triangular_mf(self.kd_universe, 50, 100, 150),
            'ZO': self._triangular_mf(self.kd_universe, 100, 150, 200),
            'PS': self._triangular_mf(self.kd_universe, 150, 200, 250),
            'PM': self._triangular_mf(self.kd_universe, 200, 250, 300),
            'PB': self._triangular_mf(self.kd_universe, 250, 300, 300)
        }
    
    def _triangular_mf(self, universe, a, b, c):
        """Create triangular membership function."""
        mf = np.zeros_like(universe)
        for i, x in enumerate(universe):
            if a <= x <= b:
                mf[i] = (x - a) / (b - a) if b != a else 0
            elif b <= x <= c:
                mf[i] = (c - x) / (c - b) if c != b else 0
            else:
                mf[i] = 0
        return mf
    
    def _init_rule_base(self):
        """Initialize fuzzy rule base for Kp, Ki, Kd tuning."""
        # Rule base format: (E, CE) -> (Kp_term, Ki_term, Kd_term)
        # Based on standard fuzzy PID tuning rules
        self.rule_base = {
            # Error = NB (Negative Big)
            ('NB', 'NB'): ('PB', 'NB', 'PS'), ('NB', 'NM'): ('PB', 'NB', 'NS'),
            ('NB', 'NS'): ('PM', 'NM', 'NB'), ('NB', 'ZO'): ('PM', 'NM', 'NB'),
            ('NB', 'PS'): ('PS', 'NS', 'NB'), ('NB', 'PM'): ('ZO', 'ZO', 'NM'),
            ('NB', 'PB'): ('ZO', 'ZO', 'PS'),
            
            # Error = NM (Negative Medium)
            ('NM', 'NB'): ('PB', 'NB', 'NS'), ('NM', 'NM'): ('PB', 'NB', 'NB'),
            ('NM', 'NS'): ('PM', 'NM', 'NB'), ('NM', 'ZO'): ('PM', 'NS', 'NM'),
            ('NM', 'PS'): ('PS', 'NS', 'NM'), ('NM', 'PM'): ('ZO', 'ZO', 'NS'),
            ('NM', 'PB'): ('ZO', 'ZO', 'ZO'),
            
            # Error = NS (Negative Small)
            ('NS', 'NB'): ('PM', 'NB', 'ZO'), ('NS', 'NM'): ('PM', 'NM', 'NS'),
            ('NS', 'NS'): ('PM', 'NS', 'NM'), ('NS', 'ZO'): ('PS', 'NS', 'NM'),
            ('NS', 'PS'): ('PS', 'ZO', 'NS'), ('NS', 'PM'): ('ZO', 'ZO', 'NS'),
            ('NS', 'PB'): ('ZO', 'PS', 'ZO'),
            
            # Error = ZO (Zero)
            ('ZO', 'NB'): ('PM', 'NM', 'ZO'), ('ZO', 'NM'): ('PS', 'NS', 'PS'),
            ('ZO', 'NS'): ('PS', 'ZO', 'PS'), ('ZO', 'ZO'): ('ZO', 'ZO', 'ZO'),
            ('ZO', 'PS'): ('PS', 'ZO', 'PS'), ('ZO', 'PM'): ('PS', 'NS', 'PS'),
            ('ZO', 'PB'): ('PM', 'NM', 'ZO'),
            
            # Error = PS (Positive Small)
            ('PS', 'NB'): ('ZO', 'PS', 'ZO'), ('PS', 'NM'): ('ZO', 'ZO', 'NS'),
            ('PS', 'NS'): ('PS', 'ZO', 'NS'), ('PS', 'ZO'): ('PS', 'NS', 'NM'),
            ('PS', 'PS'): ('PS', 'NS', 'NM'), ('PS', 'PM'): ('PM', 'NS', 'NM'),
            ('PS', 'PB'): ('PM', 'NM', 'ZO'),
            
            # Error = PM (Positive Medium)
            ('PM', 'NB'): ('ZO', 'ZO', 'ZO'), ('PM', 'NM'): ('ZO', 'ZO', 'NS'),
            ('PM', 'NS'): ('PS', 'NS', 'NM'), ('PM', 'ZO'): ('PM', 'NS', 'NM'),
            ('PM', 'PS'): ('PM', 'NS', 'NB'), ('PM', 'PM'): ('PM', 'NM', 'NB'),
            ('PM', 'PB'): ('PB', 'NB', 'NS'),
            
            # Error = PB (Positive Big)
            ('PB', 'NB'): ('ZO', 'ZO', 'PS'), ('PB', 'NM'): ('ZO', 'ZO', 'NM'),
            ('PB', 'NS'): ('PS', 'NS', 'NB'), ('PB', 'ZO'): ('PM', 'NM', 'NB'),
            ('PB', 'PS'): ('PM', 'NM', 'NB'), ('PB', 'PM'): ('PB', 'NB', 'NS'),
            ('PB', 'PB'): ('PB', 'NB', 'PS')
        }
    
    def _fuzzify(self, crisp_value, universe, membership_functions):
        """Fuzzify crisp value to get membership degrees for all linguistic terms."""
        membership_degrees = {}
        for term, mf in membership_functions.items():
            # Find the membership degree by interpolation
            idx = np.searchsorted(universe, crisp_value)
            if idx == 0:
                membership_degrees[term] = mf[0]
            elif idx >= len(universe):
                membership_degrees[term] = mf[-1]
            else:
                # Linear interpolation
                x1, x2 = universe[idx-1], universe[idx]
                y1, y2 = mf[idx-1], mf[idx]
                membership_degrees[term] = y1 + (y2 - y1) * (crisp_value - x1) / (x2 - x1)
        return membership_degrees
    
    def _fuzzy_inference(self, e_degrees, ce_degrees):
        """Perform fuzzy inference using Mamdani method."""
        # Initialize output membership functions
        kp_output = np.zeros_like(self.kp_universe)
        ki_output = np.zeros_like(self.ki_universe)
        kd_output = np.zeros_like(self.kd_universe)
        
        # Apply all rules
        for (e_term, ce_term), (kp_term, ki_term, kd_term) in self.rule_base.items():
            # Get membership degrees for this rule
            e_degree = e_degrees.get(e_term, 0)
            ce_degree = ce_degrees.get(ce_term, 0)
            
            # Rule strength (min of antecedents)
            rule_strength = min(e_degree, ce_degree)
            
            if rule_strength > 0:
                # Apply rule to outputs using min operator
                kp_output = np.maximum(kp_output, np.minimum(rule_strength, self.kp_mf[kp_term]))
                ki_output = np.maximum(ki_output, np.minimum(rule_strength, self.ki_mf[ki_term]))
                kd_output = np.maximum(kd_output, np.minimum(rule_strength, self.kd_mf[kd_term]))
        
        return kp_output, ki_output, kd_output
    
    def _defuzzify(self, fuzzy_output, universe):
        """Defuzzify fuzzy output using centroid method."""
        if np.sum(fuzzy_output) == 0:
            return 0.0
        
        # Centroid defuzzification
        centroid = np.sum(fuzzy_output * universe) / np.sum(fuzzy_output)
        return centroid
    
    def update(self, process_variable, dt):
        """
        Update the fuzzy PID controller and compute output.
        
        Args:
            process_variable (float): Current process variable (tank level)
            dt (float): Time step
            
        Returns:
            float: Controller output
        """
        if dt <= 0:
            return self._last_output
        
        # Calculate error and change of error
        error = self.set_point - process_variable
        change_of_error = (error - self._last_error) / dt if dt > 0 else 0
        
        # Limit the inputs to universe of discourse
        error = np.clip(error, self.error_universe[0], self.error_universe[-1])
        change_of_error = np.clip(change_of_error, self.ce_universe[0], self.ce_universe[-1])
        
        # Step 1: Fuzzification
        e_degrees = self._fuzzify(error, self.error_universe, self.e_mf)
        ce_degrees = self._fuzzify(change_of_error, self.ce_universe, self.ce_mf)
        
        # Step 2: Fuzzy Inference
        kp_fuzzy, ki_fuzzy, kd_fuzzy = self._fuzzy_inference(e_degrees, ce_degrees)
        
        # Step 3: Defuzzification
        Kp = self._defuzzify(kp_fuzzy, self.kp_universe)
        Ki = self._defuzzify(ki_fuzzy, self.ki_universe)
        Kd = self._defuzzify(kd_fuzzy, self.kd_universe)
        
        # Step 4: PID computation with dynamic gains
        # Proportional term
        proportional = Kp * error
        
        # Integral term
        self._integral += Ki * error * dt
        # Anti-windup: limit integral term
        self._integral = np.clip(self._integral, self.output_min, self.output_max)
        integral = self._integral
        
        # Derivative term
        derivative = Kd * change_of_error
        
        # Total output
        output = proportional + integral + derivative
        
        # Update state
        self._last_error = error
        self._last_output = np.clip(output, self.output_min, self.output_max)
        
        return self._last_output
    
    def set_setpoint(self, set_point):
        """Update setpoint and reset controller state."""
        self.set_point = set_point
        self._last_error = 0
        self._last_output = 0
        self._integral = 0
    
    def reset(self):
        """Reset controller state."""
        self._last_error = 0
        self._last_output = 0
        self._integral = 0


# --- LỚP HỆ THỐNG BỒN NƯỚC ĐÔI ---
class CoupledTankSystem:
    """
    Mô phỏng hệ thống vật lý của hai bồn nước được kết nối.
    Các phương trình dựa trên tài liệu được cung cấp.
    """
    def __init__(self):
        # Các tham số từ Bảng I trong tài liệu
        self.A1 = 32.0  # cm^2
        self.A2 = 32.0  # cm^2
        self.alpha1 = 14.30  # cm^(3/2)/sec
        self.alpha2 = 14.30  # cm^(3/2)/sec
        self.alpha3 = 20.00  # cm^(3/2)/sec
        
        # Trạng thái ban đầu
        self.H1 = 0.0  # Mực nước trong bồn 1 (cm)
        self.H2 = 0.0  # Mực nước trong bồn 2 (cm)
        self.max_height = 40.0 # Chiều cao tối đa của bồn (cm)
        
        # Thêm các tham số cho van điều khiển
        self.valve1_open = 100.0  # Độ mở van 1 (%)
        self.valve2_open = 100.0  # Độ mở van 2 (%)
        
        # Tham số cho nhiễu loạn
        self.disturbance_active = False
        self.disturbance_start_time = 0.0
        self.disturbance_duration = 5.0  # Thời gian nhiễu (giây)
        self.disturbance_flow = 50.0  # Lưu lượng nhiễu (cm³/s)

    def update(self, Qi1, Qi2, dt, current_time=0.0):
        """
        Cập nhật trạng thái của các bồn nước qua một khoảng thời gian dt.

        Args:
            Qi1 (float): Lưu lượng dòng chảy vào bồn 1 (cm^3/s).
            Qi2 (float): Lưu lượng dòng chảy vào bồn 2 (thường là nhiễu, ở đây là 0).
            dt (float): Khoảng thời gian (s).
            current_time (float): Thời gian hiện tại để xử lý nhiễu.
        """
        # Đảm bảo mực nước không âm
        H1_safe = max(0, self.H1)
        H2_safe = max(0, self.H2)

        # Tính toán các lưu lượng ra dựa trên phương trình Bernoulli và độ mở van
        Qo1 = 0.0 if self.valve1_open < 1e-3 else (self.valve1_open / 100.0) * self.alpha1 * math.sqrt(H1_safe)
        Qo2 = 0.0 if self.valve2_open < 1e-3 else (self.valve2_open / 100.0) * self.alpha2 * math.sqrt(H2_safe)
        
        # Lưu lượng giữa hai bồn, xử lý cả hai chiều
        delta_H = self.H1 - self.H2
        if delta_H > 0:
            Qo3 = self.alpha3 * math.sqrt(delta_H)
        else:
            Qo3 = -self.alpha3 * math.sqrt(-delta_H)

        # Tính toán sự thay đổi mực nước (dH/dt) dựa trên phương trình (1) và (2)
        dH1_dt = (Qi1 - Qo1 - Qo3) / self.A1
        dH2_dt = (Qi2 - Qo2 + Qo3) / self.A2
        
        # Xử lý nhiễu loạn
        if self.disturbance_active:
            if current_time - self.disturbance_start_time < self.disturbance_duration:
                # Trừ lưu lượng nhiễu từ bồn 2
                dH2_dt -= (self.disturbance_flow / self.A2)
            else:
                # Kết thúc nhiễu
                self.disturbance_active = False

        # Cập nhật mực nước bằng phương pháp Euler
        self.H1 += dH1_dt * dt
        self.H2 += dH2_dt * dt

        # Giữ mực nước trong giới hạn của bồn
        self.H1 = max(0, min(self.H1, self.max_height))
        self.H2 = max(0, min(self.H2, self.max_height))

    def set_valve_openings(self, valve1_open, valve2_open):
        """
        Đặt độ mở của các van.
        
        Args:
            valve1_open (float): Độ mở van 1 (0-100%)
            valve2_open (float): Độ mở van 2 (0-100%)
        """
        self.valve1_open = max(0, min(100, valve1_open))
        self.valve2_open = max(0, min(100, valve2_open))

    def trigger_disturbance(self, current_time):
        """
        Kích hoạt nhiễu loạn.
        
        Args:
            current_time (float): Thời gian hiện tại
        """
        self.disturbance_active = True
        self.disturbance_start_time = current_time

    def get_levels(self):
        """Trả về mực nước hiện tại."""
        return self.H1, self.H2
        
    def reset(self):
        """Đặt lại mực nước về 0."""
        self.H1 = 0.0
        self.H2 = 0.0
        self.disturbance_active = False


# --- LỚP GIAO DIỆN NGƯỜI DÙNG ---
class SimulationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Nền tảng Phân tích Điều khiển Bồn Nước Đôi")
        self.root.geometry("1200x900")
        
        self.is_running = False
        self.simulation_speed = 1
        self.dt = 0.1
        self.simulation_time = 0.0
        self.graph_time_window = 30.0

        self.tank_system = CoupledTankSystem()
        output_limits = (0, 300) 
        self.pid_controller = PIDController(Kp=83.5, Ki=14.5, Kd=120, set_point=25.0, output_limits=output_limits)
        self.fuzzy_controller = FuzzyPIDController(set_point=self.pid_controller.set_point, output_limits=output_limits)
        self.active_controller = self.pid_controller # Mặc định là PID truyền thống

        # Kích thước bồn nước (để sử dụng trong các phương thức khác) - Dùng giá trị ban đầu, sẽ tính lại trong _redraw_canvas
        self.tank_width = 150
        self.tank_height = 300
        self.tank1_x0 = 50
        self.tank1_y0 = 50
        self.tank2_x0 = self.tank1_x0 + self.tank_width + 100

        # Dữ liệu cho biểu đồ
        self.time_data = []
        self.h2_data = []
        self.setpoint_data = []
        self.h1_data = [] # Thêm để xuất CSV
        self.max_data_points = 200  

        # Các đối tượng hoạt họa dòng chảy
        self.flow_particles = [] 
        self.flow_animation_id = None

        # Biến cho van điều khiển
        self.valve1_open = 100.0  
        self.valve2_open = 100.0  

        # Biến cho nhiễu loạn
        self.disturbance_button_state = False 

        # Biến cho các thanh trượt PID và Setpoint (SỬA LỖI: Di chuyển khởi tạo vào __init__)
        self.kp_var = tk.DoubleVar(value=self.pid_controller.Kp)
        self.ki_var = tk.DoubleVar(value=self.pid_controller.Ki)
        self.kd_var = tk.DoubleVar(value=self.pid_controller.Kd)
        self.setpoint_var = tk.DoubleVar(value=self.pid_controller.set_point)
        self.relay_amplitude_var = tk.DoubleVar(value=100.0)  # Giá trị mặc định là 100

        # Biến cho Auto-tuning (Module B) - Relay Method
        self.auto_tuning_active = False
        self.auto_tuning_Ku = 0.0
        self.auto_tuning_Tu = 0.0
        self.initial_autotune_setpoint = 20.0  # Setpoint cho quá trình autotune
        
        # Relay Method variables
        self.relay_amplitude = 100.0  # Biên độ relay (cm³/s)
        self.relay_state = 'off'  # Trạng thái relay ('on' hoặc 'off')
        self.cycle_count = 0  # Đếm số chu kỳ để bỏ qua giai đoạn quá độ
        self.relay_peaks = []  # Lưu các đỉnh của dao động relay
        self.relay_troughs = []  # Lưu các đáy của dao động relay
        self.relay_start_time = 0.0  # Thời gian bắt đầu relay
        self.transient_cycles = 3  # Số chu kỳ quá độ cần bỏ qua
        self.measurement_cycles = 4  # Số chu kỳ để đo lường
        self.AUTOTUNE_TIMEOUT_SECONDS = 200.0  # Thời gian tối đa cho quá trình tinh chỉnh

        # Đối tượng canvas GUI (Sẽ được tạo trong _create_operate_tab)
        self.canvas = None
        
        # Các đối tượng vẽ trên canvas (khởi tạo None để tránh lỗi AttributeErrors khi truy cập sớm)
        self.water1_rect = None
        self.water2_rect = None
        self.setpoint_line = None
        self.valve1_rect = None
        self.valve2_rect = None

        # Thiết lập giao diện
        self._create_menu_bar()

        # Notebook (Tabbed Interface)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Tabs
        self.tab_operate = ttk.Frame(self.notebook)
        self.tab_pid = ttk.Frame(self.notebook)
        self.tab_fuzzy = ttk.Frame(self.notebook)
        
        self.notebook.add(self.tab_operate, text="Vận hành")
        self.notebook.add(self.tab_pid, text="Tinh chỉnh PID")
        self.notebook.add(self.tab_fuzzy, text="Thiết lập Logic Mờ")

        # Tạo nội dung cho từng tab
        self._create_operate_tab(self.tab_operate)
        self._create_pid_tab(self.tab_pid)
        self._create_fuzzy_tab(self.tab_fuzzy)

        # Bắt đầu vòng lặp cập nhật GUI
        # SỬA LỖI: Hoãn gọi update_gui để đảm bảo GUI đã được render đầy đủ
        self.root.after(100, self.update_gui)

    def _create_menu_bar(self):
        menubar = tk.Menu(self.root)
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Xuất dữ liệu ra CSV...", command=self.export_csv)
        menubar.add_cascade(label="Tệp", menu=filemenu)
        self.root.config(menu=menubar)

    def export_csv(self):
        """Xuất dữ liệu mô phỏng ra file CSV."""
        if self.is_running:
            self.stop_simulation()
            messagebox.showinfo("Thông báo", "Mô phỏng đã được tạm dừng để xuất dữ liệu.")

        if not self.time_data:
            messagebox.showwarning("Không có dữ liệu", "Không có dữ liệu để xuất. Hãy chạy mô phỏng trước.")
            return

        try:
            filepath = asksaveasfilename(
                initialdir=os.getcwd(),
                defaultextension=".csv",
                filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
                title="Lưu dữ liệu mô phỏng"
            )
            if not filepath:
                return

            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['Time (s)', 'Setpoint (cm)', 'H1 (cm)', 'H2 (cm)'])
                
                for i in range(len(self.time_data)):
                    writer.writerow([
                        round(self.time_data[i], 3),
                        round(self.setpoint_data[i], 3),
                        round(self.h1_data[i], 3),
                        round(self.h2_data[i], 3)
                    ])
            
            messagebox.showinfo("Thành công", f"Dữ liệu đã được xuất thành công tới:\n{os.path.basename(filepath)}")
        except Exception as e:
            messagebox.showerror("Lỗi", f"Đã xảy ra lỗi khi xuất file CSV: {e}")

    def draw_tanks(self):
        # Xóa toàn bộ canvas
        self.canvas.delete("all")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        # Tính toán lại vị trí/kích thước và lưu vào self.*
        # Tăng kích thước bồn nước để tận dụng không gian tốt hơn
        self.tank_width = int(w * 0.22) # Tăng từ 0.18
        self.tank_height = int(h * 0.75) # Tăng từ 0.7
        self.tank1_x0 = int(w * 0.1) # Điều chỉnh vị trí x
        self.tank1_y0 = int(h * 0.1) # Điều chỉnh vị trí y
        self.tank2_x0 = self.tank1_x0 + self.tank_width + int(w * 0.18) # Khoảng cách giữa 2 bồn
        # Vẽ bồn 1
        self.canvas.create_rectangle(self.tank1_x0, self.tank1_y0, self.tank1_x0 + self.tank_width, self.tank1_y0 + self.tank_height, outline="black", width=2)
        self.canvas.create_text(self.tank1_x0 + self.tank_width//2, self.tank1_y0 - 15, text="Bồn 1 (Tank 1)")
        # Vẽ bồn 2
        self.canvas.create_rectangle(self.tank2_x0, self.tank1_y0, self.tank2_x0 + self.tank_width, self.tank1_y0 + self.tank_height, outline="black", width=2)
        self.canvas.create_text(self.tank2_x0 + self.tank_width//2, self.tank1_y0 - 15, text="Bồn 2 (Tank 2)")
        # Ống nối
        self.pipe_y = self.tank1_y0 + int(self.tank_height * 0.75) # Lưu pipe_y
        self.canvas.create_line(self.tank1_x0 + self.tank_width, self.pipe_y, self.tank2_x0, self.pipe_y, width=10, fill="gray")
        # Ống ra
        self.out_pipe_y = self.tank1_y0 + self.tank_height - 10 # Lưu out_pipe_y
        self.canvas.create_line(self.tank1_x0, self.out_pipe_y, self.tank1_x0 - 30, self.out_pipe_y + 10, width=8, fill="gray")
        self.canvas.create_line(self.tank2_x0, self.out_pipe_y, self.tank2_x0 - 30, self.out_pipe_y + 10, width=8, fill="gray")
        # Van (Cải thiện hiển thị van, dùng hình chữ nhật và vị trí chính xác)
        valve_size = 25 # Kích thước van
        
        def valve_color(opening):
            if opening < 10:
                return "#d32f2f"
            elif opening < 70:
                return "#fbc02d"
            else:
                return "#388e3c"
        
        valve1_val = self.valve1_var.get() if hasattr(self, 'valve1_var') else self.valve1_open
        valve2_val = self.valve2_var.get() if hasattr(self, 'valve2_var') else self.valve2_open

        # Tính toán tọa độ trung tâm của van (cuối đường ống thoát nước)
        valve1_center_x = self.tank1_x0 - 30
        valve1_center_y = self.out_pipe_y + 10
        valve2_center_x = self.tank2_x0 - 30
        valve2_center_y = self.out_pipe_y + 10

        # Van 1 (vẽ hình chữ nhật)
        self.valve1_rect = self.canvas.create_rectangle(
            valve1_center_x - valve_size/2, valve1_center_y - valve_size/2,
            valve1_center_x + valve_size/2, valve1_center_y + valve_size/2,
            fill=valve_color(valve1_val), outline="black", width=2
        )
        self.canvas.create_text(valve1_center_x, valve1_center_y + valve_size, text="V1", font=("Arial", 12, "bold"))

        # Van 2 (vẽ hình chữ nhật)
        self.valve2_rect = self.canvas.create_rectangle(
            valve2_center_x - valve_size/2, valve2_center_y - valve_size/2,
            valve2_center_x + valve_size/2, valve2_center_y + valve_size/2,
            fill=valve_color(valve2_val), outline="black", width=2
        )
        self.canvas.create_text(valve2_center_x, valve2_center_y + valve_size, text="V2", font=("Arial", 12, "bold"))

        # Ống vào
        self.canvas.create_line(self.tank1_x0 + 20, self.tank1_y0, self.tank1_x0 + 20, self.tank1_y0 - 30, width=8, fill="gray", arrow=tk.LAST)
        self.canvas.create_text(self.tank1_x0 + 20, self.tank1_y0 - 40, text="Qi1")
        # Mực nước
        h1, h2 = self.tank_system.get_levels()
        level1_px = (h1 / self.tank_system.max_height) * self.tank_height
        level2_px = (h2 / self.tank_system.max_height) * self.tank_height
        y1 = self.tank1_y0 + self.tank_height - level1_px
        y2 = self.tank1_y0 + self.tank_height - level2_px
        self.water1_rect = self.canvas.create_rectangle(self.tank1_x0, y1, self.tank1_x0 + self.tank_width, self.tank1_y0 + self.tank_height, fill="lightblue", outline="")
        self.water2_rect = self.canvas.create_rectangle(self.tank2_x0, y2, self.tank2_x0 + self.tank_width, self.tank1_y0 + self.tank_height, fill="lightblue", outline="")
        # Đường setpoint
        setpoint_h = self.setpoint_var.get()
        setpoint_px = (setpoint_h / self.tank_system.max_height) * self.tank_height
        setpoint_y = self.tank1_y0 + self.tank_height - setpoint_px
        self.setpoint_line = self.canvas.create_line(self.tank2_x0 - 5, setpoint_y, self.tank2_x0 + self.tank_width + 5, setpoint_y, fill="red", dash=(4, 2), width=2)
        # Vẽ lại các hạt nước động (chỉ cập nhật vị trí, không tạo mới)
        for particle in self.flow_particles:
            if not particle['active']:
                continue
            # Cần tính toán lại vị trí tương đối của hạt nước nếu muốn chúng scale theo canvas
            # Hiện tại, chỉ cập nhật tọa độ tuyệt đối mà chúng đã có
            self.canvas.coords(
                particle['id'],
                particle['x']-2, particle['y']-2,
                particle['x']+2, particle['y']+2
            )

    def _create_operate_tab(self, tab):
        # Configure the tab to expand to fill the notebook
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Main frame for the operate tab
        main_frame = ttk.Frame(tab, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew") # Use grid instead of pack

        # Configure main_frame columns and rows to expand
        main_frame.grid_columnconfigure(0, weight=99) # Canvas column (MAXIMUM space)
        main_frame.grid_columnconfigure(1, weight=1) # Controls column
        main_frame.grid_rowconfigure(0, weight=2) # Canvas/Controls row
        main_frame.grid_rowconfigure(1, weight=1) # Info frame row
        main_frame.grid_rowconfigure(2, weight=3) # Graph/KPIs row

        # --- Canvas frame to draw tanks ---
        canvas_frame = ttk.LabelFrame(main_frame, text="Sơ đồ Hệ thống", padding="10")
        canvas_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        # Configure canvas_frame to expand
        canvas_frame.grid_rowconfigure(0, weight=1)
        canvas_frame.grid_columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, bg="white", bd=2, relief="groove")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        
        # Gắn sự kiện cấu hình để vẽ lại bồn nước khi thay đổi kích thước
        self.canvas.bind("<Configure>", lambda e: self.draw_tanks())
        
        # --- Controls frame for this tab (Start/Stop/Reset/Disturbance) ---
        controls_frame = ttk.LabelFrame(main_frame, text="Điều khiển Chung", padding="10")
        controls_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        controls_frame.grid_columnconfigure(0, weight=1)

        button_row = 0
        # Nút điều khiển mô phỏng
        ttk.Label(controls_frame, text="Điều khiển Mô phỏng:").grid(row=button_row, column=0, sticky=tk.W, pady=5)
        button_frame = ttk.Frame(controls_frame)
        button_frame.grid(row=button_row+1, column=0, sticky=tk.NSEW, pady=5)
        self.start_button = ttk.Button(button_frame, text="Bắt đầu", command=self.start_simulation)
        self.start_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.stop_button = ttk.Button(button_frame, text="Tạm dừng", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        self.reset_button = ttk.Button(button_frame, text="Reset", command=self.reset_simulation)
        self.reset_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        
        button_row += 2
        # Nút tạo nhiễu
        ttk.Label(controls_frame, text="Tạo Nhiễu Loạn:").grid(row=button_row, column=0, sticky=tk.W, pady=5)
        self.disturbance_button = ttk.Button(controls_frame, text="Tạo Nhiễu", command=self.trigger_disturbance)
        self.disturbance_button.grid(row=button_row+1, column=0, sticky=tk.NSEW, pady=5, padx=2)

        # --- Information Display ---
        info_frame = ttk.LabelFrame(main_frame, text="Thông tin Trạng thái", padding="10")
        info_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
        # Configure info_frame columns for better distribution of labels
        info_frame.grid_columnconfigure(0, weight=1) # H1 label
        info_frame.grid_columnconfigure(1, weight=1) # H2 label
        info_frame.grid_columnconfigure(2, weight=2) # Qi1 label (longer text, give more space)
        
        self.h1_label = ttk.Label(info_frame, text="Mực nước H1: 0.00 cm")
        self.h1_label.grid(row=0, column=0, padx=8, pady=4, sticky="nsew")
        self.h2_label = ttk.Label(info_frame, text="Mực nước H2: 0.00 cm")
        self.h2_label.grid(row=0, column=1, padx=8, pady=4, sticky="nsew")
        self.qi1_label = ttk.Label(info_frame, text="Lưu lượng vào Qi1: 0.00 cm³/s")
        self.qi1_label.grid(row=0, column=2, padx=8, pady=4, sticky="nsew")

        # --- Graph and KPIs frame ---
        graph_kpi_frame = ttk.Frame(main_frame)
        graph_kpi_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
        graph_kpi_frame.grid_rowconfigure(0, weight=3)
        graph_kpi_frame.grid_rowconfigure(1, weight=1)

        graph_frame = ttk.LabelFrame(graph_kpi_frame, text="Biểu đồ Hiệu năng PID", padding="10")
        graph_frame.grid(row=0, column=0, sticky="nsew")
        graph_frame.grid_rowconfigure(0, weight=1)
        graph_frame.grid_columnconfigure(0, weight=1)

        self._setup_graph(graph_frame)

        # KPIs frame
        self.kpi_frame = ttk.LabelFrame(graph_kpi_frame, text="Các chỉ số Hiệu năng (KPIs)", padding="10")
        self.kpi_frame.grid(row=1, column=0, sticky="nsew")
        # Tạm thời để trống, sẽ điền sau

    def _create_pid_tab(self, tab):
        controls_frame = ttk.LabelFrame(tab, text="Bảng Điều Khiển PID", padding="10")
        controls_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        # Configure grid columns for better alignment
        controls_frame.grid_columnconfigure(0, weight=0) # Labels column
        controls_frame.grid_columnconfigure(1, weight=5) # Sliders column (give more space)
        controls_frame.grid_columnconfigure(2, weight=0) # Value labels column

        row_idx = 0
        # Combobox chọn bộ điều khiển
        ttk.Label(controls_frame, text="Chọn Bộ Điều Khiển:").grid(row=row_idx, column=0, sticky=tk.W, pady=5, padx=5)
        self.controller_var = tk.StringVar(value="PID Truyền Thống")
        self.controller_combo = ttk.Combobox(controls_frame, textvariable=self.controller_var, state="readonly", 
                                             values=["PID Truyền Thống", "PID Logic Mờ"])
        self.controller_combo.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), padx=5, pady=5, columnspan=2) # Span across slider and value columns
        self.controller_combo.bind("<<ComboboxSelected>>", self._on_controller_change)
        row_idx += 1

        # Các thanh trượt PID
        ttk.Label(controls_frame, text="Hệ số Kp:").grid(row=row_idx, column=0, sticky=tk.W, pady=2, padx=5)
        ttk.Scale(controls_frame, from_=0, to=200, orient=tk.HORIZONTAL, variable=self.kp_var, command=self.update_pid_gains).grid(row=row_idx, column=1, sticky=tk.EW, padx=5, pady=2)
        self.kp_label = ttk.Label(controls_frame, text=f"{self.kp_var.get():.1f}")
        self.kp_label.grid(row=row_idx, column=2, padx=5)
        row_idx += 1

        ttk.Label(controls_frame, text="Hệ số Ki:").grid(row=row_idx, column=0, sticky=tk.W, pady=2, padx=5)
        ttk.Scale(controls_frame, from_=0, to=50, orient=tk.HORIZONTAL, variable=self.ki_var, command=self.update_pid_gains).grid(row=row_idx, column=1, sticky=tk.EW, padx=5, pady=2)
        self.ki_label = ttk.Label(controls_frame, text=f"{self.ki_var.get():.1f}")
        self.ki_label.grid(row=row_idx, column=2, padx=5)
        row_idx += 1

        ttk.Label(controls_frame, text="Hệ số Kd:").grid(row=row_idx, column=0, sticky=tk.W, pady=2, padx=5)
        ttk.Scale(controls_frame, from_=0, to=300, orient=tk.HORIZONTAL, variable=self.kd_var, command=self.update_pid_gains).grid(row=row_idx, column=1, sticky=tk.EW, padx=5, pady=2)
        self.kd_label = ttk.Label(controls_frame, text=f"{self.kd_var.get():.1f}")
        self.kd_label.grid(row=row_idx, column=2, padx=5)
        row_idx += 1

        ttk.Label(controls_frame, text="Mực nước mong muốn (Setpoint):").grid(row=row_idx, column=0, sticky=tk.W, pady=2, padx=5)
        ttk.Scale(controls_frame, from_=0, to=self.tank_system.max_height, orient=tk.HORIZONTAL, variable=self.setpoint_var, command=self.update_pid_setpoint).grid(row=row_idx, column=1, sticky=tk.EW, padx=5, pady=2)
        self.setpoint_label = ttk.Label(controls_frame, text=f"{self.setpoint_var.get():.1f} cm")
        self.setpoint_label.grid(row=row_idx, column=2, padx=5)
        row_idx += 1

        # Thanh trượt Van điều khiển
        ttk.Label(controls_frame, text="Độ mở Van 1 (%):").grid(row=row_idx, column=0, sticky=tk.W, pady=2, padx=5)
        self.valve1_var = tk.DoubleVar(value=self.valve1_open)
        ttk.Scale(controls_frame, from_=0, to=100, orient=tk.HORIZONTAL, variable=self.valve1_var, command=self.update_valve_openings).grid(row=row_idx, column=1, sticky=tk.EW, padx=5, pady=2)
        self.valve1_label = ttk.Label(controls_frame, text=f"{self.valve1_var.get():.0f}%")
        self.valve1_label.grid(row=row_idx, column=2, padx=5)
        row_idx += 1

        ttk.Label(controls_frame, text="Độ mở Van 2 (%):").grid(row=row_idx, column=0, sticky=tk.W, pady=2, padx=5)
        self.valve2_var = tk.DoubleVar(value=self.valve2_open)
        ttk.Scale(controls_frame, from_=0, to=100, orient=tk.HORIZONTAL, variable=self.valve2_var, command=self.update_valve_openings).grid(row=row_idx, column=1, sticky=tk.EW, padx=5, pady=2)
        self.valve2_label = ttk.Label(controls_frame, text=f"{self.valve2_var.get():.0f}%")
        self.valve2_label.grid(row=row_idx, column=2, padx=5)
        row_idx += 1

        # Biên độ Relay cho auto-tuning
        ttk.Label(controls_frame, text="Biên độ Relay (d):").grid(row=row_idx, column=0, sticky=tk.W, pady=2, padx=5)
        relay_spinbox = ttk.Spinbox(controls_frame, from_=10.0, to=300.0, increment=10.0, textvariable=self.relay_amplitude_var, width=10)
        relay_spinbox.grid(row=row_idx, column=1, sticky=tk.W, padx=5, pady=2)
        row_idx += 1

        # Nút tự động tinh chỉnh (Module B)
        ttk.Separator(controls_frame, orient=tk.HORIZONTAL).grid(row=row_idx, columnspan=3, sticky=tk.EW, pady=10)
        row_idx += 1
        self.autotune_button = ttk.Button(controls_frame, text="Tự động Tinh chỉnh (Relay Method)", command=self.start_auto_tuning)
        self.autotune_button.grid(row=row_idx, column=0, columnspan=3, sticky=tk.EW, pady=5, padx=5)
        self.autotune_status_label = ttk.Label(controls_frame, text="")
        self.autotune_status_label.grid(row=row_idx+1, column=0, columnspan=3, sticky=tk.W, padx=5)

    def _create_fuzzy_tab(self, tab):
        fuzzy_frame = ttk.LabelFrame(tab, text="Bảng Luật Mờ (Kp, Ki, Kd)", padding="10")
        fuzzy_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Tạo frame chứa lưới (có thể cuộn nếu cần)
        canvas_container = tk.Canvas(fuzzy_frame)
        scrollbar_y = ttk.Scrollbar(fuzzy_frame, orient="vertical", command=canvas_container.yview)
        scrollbar_x = ttk.Scrollbar(fuzzy_frame, orient="horizontal", command=canvas_container.xview)
        scrollable_frame = ttk.Frame(canvas_container)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas_container.configure(
                scrollregion=canvas_container.bbox("all")
            )
        )

        canvas_container.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas_container.configure(yscrollcommand=scrollbar_y.set, xscrollcommand=scrollbar_x.set)

        canvas_container.pack(side="left", fill="both", expand=True)
        scrollbar_y.pack(side="right", fill="y")
        scrollbar_x.pack(side="bottom", fill="x")

        # Hiển thị bảng luật mờ dựa trên các thuật ngữ ngôn ngữ
        linguistic_terms = self.fuzzy_controller.linguistic_terms
        
        # Header row
        ttk.Label(scrollable_frame, text="Error (E) \\ CE", relief="solid", borderwidth=1, 
                 width=12, justify=tk.CENTER).grid(row=0, column=0, padx=1, pady=1)
        for j, ce_term in enumerate(linguistic_terms):
            ttk.Label(scrollable_frame, text=ce_term, relief="solid", borderwidth=1, 
                     width=12, justify=tk.CENTER).grid(row=0, column=j+1, padx=1, pady=1)
        
        # Data rows
        for i, e_term in enumerate(linguistic_terms):
            ttk.Label(scrollable_frame, text=e_term, relief="solid", borderwidth=1, 
                     width=12, justify=tk.CENTER).grid(row=i+1, column=0, padx=1, pady=1)
            for j, ce_term in enumerate(linguistic_terms):
                # Get rule output from rule base
                if (e_term, ce_term) in self.fuzzy_controller.rule_base:
                    kp_term, ki_term, kd_term = self.fuzzy_controller.rule_base[(e_term, ce_term)]
                    rule_text = f"Kp={kp_term}\nKi={ki_term}\nKd={kd_term}"
                else:
                    rule_text = "N/A"
                
                ttk.Label(scrollable_frame, text=rule_text, borderwidth=1, relief="solid", 
                         width=12, justify=tk.CENTER).grid(row=i+1, column=j+1, padx=1, pady=1)
        
        # Add explanation frame
        explanation_frame = ttk.LabelFrame(tab, text="Giải thích Thuật ngữ Ngôn ngữ", padding="10")
        explanation_frame.pack(fill=tk.X, padx=10, pady=5)
        
        explanation_text = """
        NB = Negative Big (Âm lớn), NM = Negative Medium (Âm trung bình), NS = Negative Small (Âm nhỏ)
        ZO = Zero (Không), PS = Positive Small (Dương nhỏ), PM = Positive Medium (Dương trung bình), PB = Positive Big (Dương lớn)
        
        Bảng luật mờ này định nghĩa cách điều chỉnh các hệ số Kp, Ki, Kd dựa trên sai số (E) và tốc độ thay đổi sai số (CE).
        Ví dụ: Khi E=NB (sai số âm lớn) và CE=NB (tốc độ thay đổi âm lớn), bộ điều khiển sẽ sử dụng Kp=PB, Ki=NB, Kd=PS.
        """
        
        ttk.Label(explanation_frame, text=explanation_text, justify=tk.LEFT, wraplength=800).pack(anchor=tk.W)

    def _setup_graph(self, parent_frame):
        """Thiết lập biểu đồ matplotlib."""
        # Tạo figure và axes
        self.fig = Figure(figsize=(10, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        # Thiết lập biểu đồ
        self.ax.set_xlabel('Thời gian (giây)')
        self.ax.set_ylabel('Mực nước (cm)')
        self.ax.set_title('Hiệu năng Bộ điều khiển PID - Mực nước Bồn 2')
        self.ax.grid(True, alpha=0.3)
        
        # Khởi tạo đường dữ liệu
        self.line_h2, = self.ax.plot([], [], 'b-', linewidth=2, label='Mực nước thực tế (H2)')
        self.line_setpoint, = self.ax.plot([], [], 'r--', linewidth=2, label='Giá trị mong muốn (Setpoint)')
        
        # Thiết lập legend
        self.ax.legend()
        
        # Tạo canvas matplotlib
        self.graph_canvas = FigureCanvasTkAgg(self.fig, parent_frame)
        self.graph_canvas.get_tk_widget().pack(expand=True, fill=tk.BOTH)

    def _update_graph_data(self, time, h1, h2, setpoint):
        """Cập nhật dữ liệu biểu đồ."""
        self.time_data.append(time)
        self.h1_data.append(h1)  # Đảm bảo dòng này tồn tại
        self.h2_data.append(h2)
        self.setpoint_data.append(setpoint)
        
        # Giới hạn số điểm dữ liệu để tránh lag
        if len(self.time_data) > self.max_data_points:
            self.time_data = self.time_data[-self.max_data_points:]
            self.h2_data = self.h2_data[-self.max_data_points:]
            self.setpoint_data = self.setpoint_data[-self.max_data_points:]
        
        # Cập nhật đường dữ liệu
        self.line_h2.set_data(self.time_data, self.h2_data)
        self.line_setpoint.set_data(self.time_data, self.setpoint_data)
        
        # Tự động điều chỉnh trục với cửa sổ thời gian cuộn
        if self.time_data:
            current_time = max(self.time_data)
            start_time = max(0, current_time - self.graph_time_window)
            self.ax.set_xlim(start_time, current_time)
            
            all_data = self.h2_data + self.setpoint_data
            if all_data:
                min_val = min(all_data)
                max_val = max(all_data)
                margin = (max_val - min_val) * 0.1 if max_val > min_val else 1
                self.ax.set_ylim(max(0, min_val - margin), max_val + margin)
        
        # Vẽ lại biểu đồ
        self.graph_canvas.draw_idle()

    def _create_flow_particle(self, x, y, direction='down', speed=2):
        """Tạo một hạt nước cho hoạt họa dòng chảy."""
        particle = {
            'id': self.canvas.create_oval(x-2, y-2, x+2, y+2, fill='lightblue', outline='blue'),
            'x': x,
            'y': y,
            'direction': direction,
            'speed': speed,
            'life': 50,  # Số frame tồn tại
            'active': True  # Trạng thái hoạt động
        }
        return particle

    def _reuse_particle(self, particle, x, y, direction='down', speed=2):
        """Tái sử dụng hạt nước đã tồn tại."""
        particle['x'] = x
        particle['y'] = y
        particle['direction'] = direction
        particle['speed'] = speed
        particle['life'] = 50
        particle['active'] = True
        # Đặt lại màu về mặc định
        self.canvas.itemconfig(particle['id'], fill='lightblue', outline='blue')
        # Cập nhật vị trí
        self.canvas.coords(particle['id'], x-2, y-2, x+2, y+2)

    def _animate_water_flow(self):
        """Hoạt họa dòng chảy nước."""
        if not self.is_running:
            return
        
        # Lấy dữ liệu hiện tại
        h1, h2 = self.tank_system.get_levels()
        qi1 = self.active_controller._last_output if self.is_running else 0
        
        # Tính toán lưu lượng ra
        qo1 = 0.0 if self.valve1_var.get() < 1e-3 else (self.valve1_var.get() / 100.0) * self.tank_system.alpha1 * math.sqrt(max(0, h1))
        qo2 = 0.0 if self.valve2_var.get() < 1e-3 else (self.valve2_var.get() / 100.0) * self.tank_system.alpha2 * math.sqrt(max(0, h2))
        
        # Tính toán lưu lượng giữa hai bồn
        delta_h = h1 - h2
        if abs(delta_h) > 0.1:  # Chỉ tạo dòng chảy nếu có chênh lệch đáng kể
            qo3 = self.tank_system.alpha3 * math.sqrt(abs(delta_h))
        else:
            qo3 = 0
        
        # Tạo hạt nước cho dòng chảy vào (Qi1)
        if qi1 > 10:  # Chỉ tạo hạt khi có lưu lượng đáng kể
            flow_intensity = min(qi1 / 100, 1.0)  # Chuẩn hóa từ 0-1
            if np.random.random() < flow_intensity * 0.3:  # Tần suất tạo hạt
                x = self.tank1_x0 + 20 + np.random.uniform(-5, 5)
                y = self.tank1_y0 - 10
                speed = 1 + flow_intensity * 3
                
                # Tìm hạt không hoạt động để tái sử dụng
                inactive_particle = None
                for particle in self.flow_particles:
                    if not particle['active']:
                        inactive_particle = particle
                        break
                
                if inactive_particle:
                    self._reuse_particle(inactive_particle, x, y, 'down', speed)
                else:
                    particle = self._create_flow_particle(x, y, 'down', speed)
                    self.flow_particles.append(particle)
        
        # Tạo hạt nước cho dòng chảy ra Qo1 (SỬA LỖI: Điều chỉnh tọa độ x và y)
        if qo1 > 5 and self.valve1_var.get() > 0:
            if np.random.random() < 0.2:
                x = self.tank1_x0 - 30 + np.random.uniform(-3, 3) # Vị trí cuối ống ra của bồn 1
                y = self.out_pipe_y + 10 + np.random.uniform(-3, 3) # Y tại ống ra
                # Tìm hạt không hoạt động để tái sử dụng
                inactive_particle = None
                for particle in self.flow_particles:
                    if not particle['active']:
                        inactive_particle = particle
                        break
                if inactive_particle:
                    self._reuse_particle(inactive_particle, x, y, 'left', 2)
                else:
                    particle = self._create_flow_particle(x, y, 'left', 2)
                    self.flow_particles.append(particle)
        
        # Tạo hạt nước cho dòng chảy ra Qo2 (SỬA LỖI: Điều chỉnh tọa độ x và y)
        if qo2 > 5 and self.valve2_var.get() > 0:
            if np.random.random() < 0.2:
                x = self.tank2_x0 - 30 + np.random.uniform(-3, 3) # Vị trí cuối ống ra của bồn 2
                y = self.out_pipe_y + 10 + np.random.uniform(-3, 3) # Y tại ống ra
                # Tìm hạt không hoạt động để tái sử dụng
                inactive_particle = None
                for particle in self.flow_particles:
                    if not particle['active']:
                        inactive_particle = particle
                        break
                if inactive_particle:
                    self._reuse_particle(inactive_particle, x, y, 'left', 2)
                else:
                    particle = self._create_flow_particle(x, y, 'left', 2)
                    self.flow_particles.append(particle)
        
        # Tạo hạt nước cho dòng chảy giữa hai bồn (Qo3)
        if qo3 > 5:
            # Sử dụng self.pipe_y đã lưu
            flow_intensity = min(qo3 / 50, 1.0)
            
            if np.random.random() < flow_intensity * 0.4: # Tần suất tạo hạt
                if delta_h > 0:  # Từ bồn 1 sang bồn 2
                    x = self.tank1_x0 + self.tank_width + 5
                    direction = 'right'
                else:  # Từ bồn 2 sang bồn 1
                    x = self.tank2_x0 - 5
                    direction = 'left'
                
                y = self.pipe_y + np.random.uniform(-5, 5) # Y tại ống nối
                speed = 1 + flow_intensity * 2
                
                # Tìm hạt không hoạt động để tái sử dụng
                inactive_particle = None
                for particle in self.flow_particles:
                    if not particle['active']:
                        inactive_particle = particle
                        break
                if inactive_particle:
                    self._reuse_particle(inactive_particle, x, y, direction, speed)
                else:
                    particle = self._create_flow_particle(x, y, direction, speed)
                    self.flow_particles.append(particle)
        
        # Tạo hạt nước cho nhiễu loạn
        if self.tank_system.disturbance_active:
            if np.random.random() < 0.4:  # Tần suất cao hơn cho nhiễu
                # Tạo nhiễu từ một vị trí khác trên bồn 2
                x = self.tank2_x0 + self.tank_width//2 + np.random.uniform(-10, 10)
                y = self.tank1_y0 + self.tank_height * 0.5 + np.random.uniform(-5, 5)
                speed = 3 # Tốc độ nhanh hơn
                
                # Tìm hạt không hoạt động để tái sử dụng
                inactive_particle = None
                for particle in self.flow_particles:
                    if not particle['active']:
                        inactive_particle = particle
                        break
                if inactive_particle:
                    self._reuse_particle(inactive_particle, x, y, 'down', speed)
                    self.canvas.itemconfig(inactive_particle['id'], fill='red', outline='darkred')
                else:
                    particle = self._create_flow_particle(x, y, 'down', speed)
                    # Đổi màu hạt nhiễu thành đỏ
                    self.canvas.itemconfig(particle['id'], fill='red', outline='darkred')
                    self.flow_particles.append(particle)
        
        # Cập nhật vị trí các hạt nước
        for particle in self.flow_particles:
            if not particle['active']:
                continue
                
            particle['life'] -= 1
            
            # Di chuyển hạt theo hướng
            if particle['direction'] == 'down':
                particle['y'] += particle['speed']
            elif particle['direction'] == 'left':
                particle['x'] -= particle['speed']
            elif particle['direction'] == 'right':
                particle['x'] += particle['speed']
            
            # Cập nhật vị trí trên canvas
            self.canvas.coords(particle['id'], 
                             particle['x']-2, particle['y']-2, 
                             particle['x']+2, particle['y']+2)
            
            # Đánh dấu hạt không hoạt động nếu hết tuổi thọ hoặc ra khỏi màn hình
            if (particle['life'] <= 0 or 
                particle['y'] > self.tank1_y0 + self.tank_height + 50 or
                particle['x'] < self.tank1_x0 - 100 or particle['x'] > self.tank2_x0 + self.tank_width + 100):
                particle['active'] = False
                # Ẩn hạt thay vì xóa
                self.canvas.coords(particle['id'], 0, 0, 0, 0)
        
        # Lên lịch cho frame tiếp theo
        self.flow_animation_id = self.root.after(50, self._animate_water_flow)

    def update_gui(self):
        """Vòng lặp chính để cập nhật GUI và chạy mô phỏng."""
        if self.is_running:
            self.run_simulation_step()
        
        # Cập nhật các nhãn giá trị
        self.kp_label.config(text=f"{self.kp_var.get():.1f}")
        self.ki_label.config(text=f"{self.ki_var.get():.1f}")
        self.kd_label.config(text=f"{self.kd_var.get():.1f}")
        self.setpoint_label.config(text=f"{self.setpoint_var.get():.1f} cm")
        
        # Cập nhật nhãn van
        self.valve1_label.config(text=f"{self.valve1_var.get():.0f}%")
        self.valve2_label.config(text=f"{self.valve2_var.get():.0f}%")
        
        # Cập nhật trạng thái nút nhiễu
        if self.tank_system.disturbance_active:
            self.disturbance_button.config(text="Nhiễu Đang Hoạt Động", state=tk.DISABLED)
        else:
            self.disturbance_button.config(text="Tạo Nhiễu", state=tk.NORMAL)

        # Lấy mực nước hiện tại
        h1, h2 = self.tank_system.get_levels()
        
        # Cập nhật nhãn thông tin
        self.h1_label.config(text=f"Mực nước H1: {h1:.2f} cm")
        self.h2_label.config(text=f"Mực nước H2: {h2:.2f} cm")
        qi1 = self.active_controller._last_output if self.is_running else 0
        self.qi1_label.config(text=f"Lưu lượng vào Qi1: {qi1:.2f} cm³/s")
        
        # Cập nhật hình ảnh trên canvas
        self.update_water_display(h1, h2)
        
        # Cập nhật biểu đồ
        if self.is_running:
            self._update_graph_data(self.simulation_time, h1, h2, self.setpoint_var.get())
        
        # Bắt đầu hoạt họa dòng chảy nếu chưa có
        if self.is_running and self.flow_animation_id is None:
            self._animate_water_flow()

        # Lên lịch cho lần cập nhật tiếp theo
        self.root.after(30, self.update_gui)

    def run_simulation_step(self):
        """Thực hiện một bước của mô phỏng vật lý."""
        for _ in range(self.simulation_speed):
            # Kiểm tra nếu đang trong quá trình auto-tuning
            if self.auto_tuning_active:
                self._run_relay_tuning_step()
            else:
                # Lấy giá trị mực nước hiện tại của bồn 2 (biến quá trình)
                current_h2 = self.tank_system.H2
                
                # PID tính toán lưu lượng vào Qi1
                qi1 = self.active_controller.update(current_h2, self.dt)
                
                # Cập nhật hệ thống bồn nước với thời gian hiện tại
                # Qi2 được đặt là 0 (không có nhiễu)
                self.tank_system.update(qi1, 0, self.dt, self.simulation_time)
            
            # Cập nhật thời gian mô phỏng
            self.simulation_time += self.dt

    def update_water_display(self, h1, h2):
        """Cập nhật lại hình chữ nhật biểu diễn mực nước trên canvas."""
        # Kiểm tra an toàn - chỉ cập nhật nếu các đối tượng đã được tạo
        if not hasattr(self, 'canvas') or self.canvas is None:
            return
            
        # SỬA: dùng lại các biến động đã lưu
        tank1_x0 = self.tank1_x0
        tank1_y0 = self.tank1_y0
        tank2_x0 = self.tank2_x0
        tank_width = self.tank_width
        tank_height = self.tank_height
        # Mực nước động
        level1_px = (h1 / self.tank_system.max_height) * tank_height
        level2_px = (h2 / self.tank_system.max_height) * tank_height
        y1 = tank1_y0 + tank_height - level1_px
        y2 = tank1_y0 + tank_height - level2_px
        
        # Kiểm tra an toàn trước khi cập nhật
        if self.water1_rect is not None:
            self.canvas.coords(self.water1_rect, tank1_x0, y1, tank1_x0 + tank_width, tank1_y0 + tank_height)
        if self.water2_rect is not None:
            self.canvas.coords(self.water2_rect, tank2_x0, y2, tank2_x0 + tank_width, tank1_y0 + tank_height)
            
        # Đường setpoint
        setpoint_h = self.setpoint_var.get()
        setpoint_px = (setpoint_h / self.tank_system.max_height) * tank_height
        setpoint_y = tank1_y0 + tank_height - setpoint_px
        
        # SỬA LỖI: Cập nhật tọa độ thay vì tạo lại
        if self.setpoint_line is not None:
            self.canvas.coords(self.setpoint_line, tank2_x0 - 5, setpoint_y, tank2_x0 + tank_width + 5, setpoint_y)
            
        # Cập nhật màu van
        self._update_valve_colors()

    def _update_valve_colors(self):
        """Cập nhật màu van theo độ mở."""
        # Kiểm tra an toàn
        if not hasattr(self, 'canvas') or self.canvas is None:
            return
            
        # Màu động
        def valve_color(opening):
            if opening < 10:
                return "#d32f2f"
            elif opening < 70:
                return "#fbc02d"
            else:
                return "#388e3c"
                
        # Kiểm tra an toàn trước khi cập nhật
        if self.valve1_rect is not None:
            self.canvas.itemconfig(self.valve1_rect, fill=valve_color(self.valve1_var.get()))
        if hasattr(self, 'valve1_label') and self.valve1_label is not None:
            self.valve1_label.config(text=f"{self.valve1_var.get():.0f}%")
            
        if self.valve2_rect is not None:
            self.canvas.itemconfig(self.valve2_rect, fill=valve_color(self.valve2_var.get()))
        if hasattr(self, 'valve2_label') and self.valve2_label is not None:
            self.valve2_label.config(text=f"{self.valve2_var.get():.0f}%")

    def update_pid_gains(self, _=None):
        """Cập nhật các hệ số PID từ thanh trượt."""
        self.active_controller.set_gains(self.kp_var.get(), self.ki_var.get(), self.kd_var.get())

    def update_pid_setpoint(self, _=None):
        """Cập nhật giá trị đặt từ thanh trượt."""
        self.active_controller.set_setpoint(self.setpoint_var.get())

    def start_simulation(self):
        self.is_running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

    def stop_simulation(self):
        self.is_running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        
        # Dừng hoạt họa dòng chảy
        if self.flow_animation_id:
            self.root.after_cancel(self.flow_animation_id)
            self.flow_animation_id = None

    def reset_simulation(self):
        self.stop_simulation()
        self.tank_system.reset()
        self.active_controller.reset()
        
        # Reset thời gian mô phỏng
        self.simulation_time = 0.0
        
        # Reset van về trạng thái mở hoàn toàn
        self.valve1_var.set(100.0)
        self.valve2_var.set(100.0)
        self.tank_system.set_valve_openings(100.0, 100.0)
        
        # Xóa dữ liệu biểu đồ
        self.time_data.clear()
        self.h1_data.clear()
        self.h2_data.clear()
        self.setpoint_data.clear()
        
        # Reset biểu đồ
        self.line_h2.set_data([], [])
        self.line_setpoint.set_data([], [])
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, self.tank_system.max_height)
        self.graph_canvas.draw_idle()
        
        # Xóa tất cả hạt nước
        for particle in self.flow_particles:
            self.canvas.delete(particle['id'])
        self.flow_particles.clear()
        
        # Đặt lại các biến để GUI cập nhật ngay lập tức
        self.h1_label.config(text="Mực nước H1: 0.00 cm")
        self.h2_label.config(text="Mực nước H2: 0.00 cm")
        self.qi1_label.config(text="Lưu lượng vào Qi1: 0.00 cm³/s")
        self.update_water_display(0,0)

    def update_valve_openings(self, _=None):
        """Cập nhật độ mở của các van từ thanh trượt."""
        self.tank_system.set_valve_openings(self.valve1_var.get(), self.valve2_var.get())

    def trigger_disturbance(self):
        """Kích hoạt nhiễu loạn."""
        self.tank_system.trigger_disturbance(self.simulation_time)

    def _on_controller_change(self, event=None):
        if self.controller_var.get() == "PID Truyền Thống":
            self.active_controller = self.pid_controller
        else:
            self.active_controller = self.fuzzy_controller

    def start_auto_tuning(self):
        if self.is_running:
            messagebox.showwarning("Cảnh báo", "Hãy dừng mô phỏng trước khi tự động tinh chỉnh.")
            return
        
        confirm = messagebox.askyesno("Xác nhận", "Bắt đầu quá trình tự động tinh chỉnh Ziegler-Nichols (Relay Method)?\nHệ thống sẽ được reset và sử dụng phương pháp relay để tìm Ku và Tu.")
        if not confirm:
            return

        self.reset_simulation()  # Reset toàn bộ hệ thống
        self.auto_tuning_active = True
        self.autotune_status_label.config(text="Đang tự động tinh chỉnh (Relay Method)...")
        self.autotune_button.config(state=tk.DISABLED)
        self.start_button.config(state=tk.DISABLED)  # Vô hiệu hóa nút Start trong khi autotune

        # Cài đặt ban đầu cho relay method
        self.setpoint_var.set(self.initial_autotune_setpoint)
        self.pid_controller.set_setpoint(self.initial_autotune_setpoint)
        
        # Lấy giá trị biên độ relay từ GUI
        self.relay_amplitude = self.relay_amplitude_var.get()
        
        # Reset relay variables
        self.relay_state = 'off'
        self.cycle_count = 0
        self.relay_peaks.clear()
        self.relay_troughs.clear()
        self.relay_start_time = self.simulation_time
        
        self.start_simulation()  # Bắt đầu mô phỏng cho quá trình autotune

    def _run_relay_tuning_step(self):
        """Thực hiện một bước của relay method để tìm Ku và Tu."""
        if not self.auto_tuning_active:
            return

        # Kiểm tra timeout
        if self.simulation_time - self.relay_start_time > self.AUTOTUNE_TIMEOUT_SECONDS:
            messagebox.showerror("Lỗi Tự động Tinh chỉnh", f"Quá trình tinh chỉnh đã vượt quá thời gian cho phép ({self.AUTOTUNE_TIMEOUT_SECONDS}s).\n\nHệ thống có thể không dao động. Hãy thử lại với giá trị 'Biên độ Relay' lớn hơn.")
            self._complete_relay_tuning()  # Gọi hàm này để dừng và reset các nút
            return

        # Lấy giá trị hiện tại
        setpoint = self.initial_autotune_setpoint
        h2 = self.tank_system.H2
        
        # Áp dụng luật relay
        if h2 < setpoint:
            # Mực nước thấp hơn setpoint -> bật relay
            if self.relay_state != 'on':
                self.relay_state = 'on'
                # Ghi nhận điểm chuyển đổi (từ off sang on)
                self._record_relay_transition('on')
            qi1 = self.relay_amplitude
        else:
            # Mực nước cao hơn hoặc bằng setpoint -> tắt relay
            if self.relay_state != 'off':
                self.relay_state = 'off'
                # Ghi nhận điểm chuyển đổi (từ on sang off)
                self._record_relay_transition('off')
            qi1 = 0.0
        
        # Cập nhật hệ thống với lưu lượng relay
        self.tank_system.update(qi1, 0, self.dt, self.simulation_time)
        
        # Kiểm tra xem đã có đủ dữ liệu để tính toán chưa
        if self._check_relay_completion():
            self._calculate_relay_parameters()
            self._apply_ziegler_nichols()
            self._complete_relay_tuning()
    
    def _record_relay_transition(self, new_state):
        """Ghi nhận điểm chuyển đổi trạng thái relay."""
        current_time = self.simulation_time
        h2 = self.tank_system.H2
        
        if new_state == 'on':
            # Chuyển từ off sang on -> đây là một đáy (trough)
            if len(self.relay_troughs) == 0 or (current_time - self.relay_troughs[-1][0]) > 0.5:
                self.relay_troughs.append((current_time, h2))
                self.cycle_count += 1
        else:  # new_state == 'off'
            # Chuyển từ on sang off -> đây là một đỉnh (peak)
            if len(self.relay_peaks) == 0 or (current_time - self.relay_peaks[-1][0]) > 0.5:
                self.relay_peaks.append((current_time, h2))
    
    def _check_relay_completion(self):
        """Kiểm tra xem đã có đủ dữ liệu để tính toán chưa."""
        # Cần ít nhất transient_cycles + measurement_cycles chu kỳ
        total_cycles_needed = self.transient_cycles + self.measurement_cycles
        
        # Kiểm tra số chu kỳ đã hoàn thành
        if self.cycle_count < total_cycles_needed:
            return False
        
        # Kiểm tra xem có đủ đỉnh và đáy để đo lường không
        if len(self.relay_peaks) < self.measurement_cycles or len(self.relay_troughs) < self.measurement_cycles:
            return False
        
        return True
    
    def _calculate_relay_parameters(self):
        """Tính toán Ku và Tu từ dữ liệu relay."""
        # Lấy các đỉnh và đáy gần nhất để đo lường
        recent_peaks = self.relay_peaks[-self.measurement_cycles:]
        recent_troughs = self.relay_troughs[-self.measurement_cycles:]
        
        # Tính chu kỳ tới hạn Tu
        peak_periods = []
        for i in range(1, len(recent_peaks)):
            period = recent_peaks[i][0] - recent_peaks[i-1][0]
            peak_periods.append(period)
        
        trough_periods = []
        for i in range(1, len(recent_troughs)):
            period = recent_troughs[i][0] - recent_troughs[i-1][0]
            trough_periods.append(period)
        
        # Tính Tu trung bình
        all_periods = peak_periods + trough_periods
        Tu = np.mean(all_periods)
        
        # Tính biên độ dao động a
        amplitudes = []
        for i in range(min(len(recent_peaks), len(recent_troughs))):
            amplitude = abs(recent_peaks[i][1] - recent_troughs[i][1])
            amplitudes.append(amplitude)
        
        a = np.mean(amplitudes)
        
        # Tính độ lợi tới hạn Ku
        d = self.relay_amplitude
        Ku = (4 * d) / (np.pi * a)
        
        # Lưu kết quả
        self.auto_tuning_Ku = Ku
        self.auto_tuning_Tu = Tu
    
    def _complete_relay_tuning(self):
        """Hoàn tất quá trình relay tuning."""
        self.auto_tuning_active = False
        
        # Hiển thị kết quả
        result_text = f"Tinh chỉnh hoàn tất! Ku={self.auto_tuning_Ku:.2f}, Tu={self.auto_tuning_Tu:.2f}"
        self.autotune_status_label.config(text=result_text)
        
        # Kích hoạt lại các nút
        self.autotune_button.config(state=tk.NORMAL)
        self.start_button.config(state=tk.NORMAL)
        
        # Dừng mô phỏng
        self.stop_simulation()
        
        # Hiển thị thông báo chi tiết
        messagebox.showinfo("Tự động tinh chỉnh", 
                           f"Quá trình tự động tinh chỉnh Ziegler-Nichols (Relay Method) đã hoàn tất!\n\n"
                           f"Kết quả:\n"
                           f"• Ku (Độ lợi tới hạn): {self.auto_tuning_Ku:.2f}\n"
                           f"• Tu (Chu kỳ tới hạn): {self.auto_tuning_Tu:.2f} s\n\n"
                           f"Các thông số PID mới đã được áp dụng.")

    def _apply_ziegler_nichols(self):
        """Áp dụng quy tắc Ziegler-Nichols để tính toán các thông số PID."""
        Ku = self.auto_tuning_Ku
        Tu = self.auto_tuning_Tu
        
        # PID không dao động (No overshoot)
        Kp = 0.33 * Ku
        Ki = 0.6 * Ku / Tu
        Kd = 0.11 * Ku * Tu

        # Cập nhật thanh trượt và PID controller
        self.kp_var.set(Kp)
        self.ki_var.set(Ki)
        self.kd_var.set(Kd)
        self.pid_controller.set_gains(Kp, Ki, Kd)


if __name__ == "__main__":
    root = tk.Tk()
    app = SimulationGUI(root)
    root.mainloop()
