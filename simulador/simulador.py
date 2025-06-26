import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class BallBeamSimulator:
    def __init__(self):
        # Parametros del sistema fisico
        self.g = 9.81  # Gravedad (m/s^2)
        self.L = 0.5   # Longitud de la barra (m)
        self.mass = 0.02  # Masa de la bola (kg)
        self.friction = 0.1  # Coeficiente de friccion
        
        # Variables del sistema
        self.ball_position = 0.3  # Posicion inicial de la bola (m)
        self.ball_velocity = 0.0  # Velocidad inicial (m/s)
        self.beam_angle = 0.0     # Angulo inicial de la barra (rad)
        
        # Parametros PID
        self.Kp = 15.0
        self.Ki = 0.1
        self.Kd = 8.0
        self.setpoint = 0.12  # 12 cm
        
        # Variables PID
        self.error = 0.0
        self.last_error = 0.0
        self.cumulative_error = 0.0
        self.error_rate = 0.0
        self.output = 0.0
        
        # Control de tiempo
        self.dt = 0.01  # 10ms como en Arduino
        self.last_time = 0
        
        # Limites del servo (en radianes)
        self.servo_min = np.radians(-35)  # 60 grados -> -35 desde horizontal
        self.servo_max = np.radians(35)   # 130 grados -> 35 desde horizontal
        
        # Arrays para datos historicos
        self.time_history = []
        self.position_history = []
        self.setpoint_history = []
        self.angle_history = []
        self.output_history = []
        
    def distance_sensor(self):
        """Simula el sensor VL53L0X"""
        # Convierte posicion en metros a centimetros
        distance_cm = self.ball_position * 100
        
        # Simula ruido del sensor
        noise = np.random.normal(0, 0.1)
        distance_cm += noise
        
        # Limita el rango del sensor
        if distance_cm < 0.3 or distance_cm > 120:
            return None  # Fuera de rango
        
        return distance_cm
    
    def limit_servo(self, angle):
        """Limita el angulo del servo"""
        return np.clip(angle, self.servo_min, self.servo_max)
    
    def pid_controller(self, measured_distance):
        """Implementa el controlador PID"""
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        
        if elapsed_time <= 0:
            return self.output
        
        # Convierte distancia a metros para el error
        measured_position = measured_distance / 100.0
        
        # Calcula error
        self.error = self.setpoint - measured_position
        
        # Termino integral
        self.cumulative_error += self.error * elapsed_time
        
        # Anti-windup
        if self.cumulative_error > 1.0:
            self.cumulative_error = 1.0
        elif self.cumulative_error < -1.0:
            self.cumulative_error = -1.0
        
        # Termino derivativo
        if elapsed_time > 0:
            self.error_rate = (self.error - self.last_error) / elapsed_time
        
        # Salida PID
        self.output = (self.Kp * self.error + 
                      self.Ki * self.cumulative_error + 
                      self.Kd * self.error_rate)
        
        # Convierte la salida a angulo del servo
        servo_angle = np.radians(self.output)
        servo_angle = self.limit_servo(servo_angle)
        
        # Actualiza variables
        self.last_error = self.error
        self.last_time = current_time
        
        return servo_angle
    
    def physics_simulation(self, dt):
        """Simula la fisica del sistema"""
        # Ecuacion de movimiento para bola en barra inclinada
        acceleration = (self.g * np.sin(self.beam_angle) - 
                       self.friction * self.ball_velocity)
        
        # Integracion numerica
        self.ball_velocity += acceleration * dt
        self.ball_position += self.ball_velocity * dt
        
        # Limites fisicos de la barra
        if self.ball_position < 0:
            self.ball_position = 0
            self.ball_velocity = 0
        elif self.ball_position > self.L:
            self.ball_position = self.L
            self.ball_velocity = 0
    
    def update_system(self):
        """Actualiza el sistema completo"""
        # Obtiene la distancia del sensor
        distance = self.distance_sensor()
        
        if distance is not None:
            # Calcula el angulo del servo usando PID
            self.beam_angle = self.pid_controller(distance)
        
        # Simula la fisica
        self.physics_simulation(self.dt)
        
        # Guarda datos
        current_time = len(self.time_history) * self.dt
        self.time_history.append(current_time)
        self.position_history.append(self.ball_position * 100)  # En cm
        self.setpoint_history.append(self.setpoint * 100)  # En cm
        self.angle_history.append(np.degrees(self.beam_angle))
        self.output_history.append(self.output)

class SimulatorGUI:
    def __init__(self):
        self.simulator = BallBeamSimulator()
        self.setup_plot()
        
    def setup_plot(self):
        """Configura la interfaz grafica"""
        self.fig, ((self.ax_system, self.ax_pos), 
                   (self.ax_angle, self.ax_output)) = plt.subplots(2, 2, figsize=(12, 8))
        
        # Configuracion del subplot del sistema
        self.ax_system.set_xlim(-0.1, 0.6)
        self.ax_system.set_ylim(-0.2, 0.2)
        self.ax_system.set_aspect('equal')
        self.ax_system.set_title('Sistema Bola y Barra')
        self.ax_system.grid(True)
        
        # Configuracion de graficas
        self.ax_pos.set_title('Posicion vs Tiempo')
        self.ax_pos.set_ylabel('Posicion (cm)')
        self.ax_pos.grid(True)
        
        self.ax_angle.set_title('Angulo del Servo')
        self.ax_angle.set_ylabel('Angulo (grados)')
        self.ax_angle.set_xlabel('Tiempo (s)')
        self.ax_angle.grid(True)
        
        self.ax_output.set_title('Salida del Controlador PID')
        self.ax_output.set_xlabel('Tiempo (s)')
        self.ax_output.set_ylabel('Salida PID')
        self.ax_output.grid(True)
        
        # Elementos graficos
        self.beam_line, = self.ax_system.plot([], [], 'b-', linewidth=6, label='Barra')
        self.ball_point, = self.ax_system.plot([], [], 'ro', markersize=12, label='Bola')
        self.setpoint_line = self.ax_system.axvline(x=0.12, color='g', linestyle='--', 
                                                   linewidth=2, label='Setpoint')
        
        # Lineas para las graficas
        self.pos_line, = self.ax_pos.plot([], [], 'b-', label='Posicion')
        self.setpoint_pos_line, = self.ax_pos.plot([], [], 'g--', label='Setpoint')
        self.angle_line, = self.ax_angle.plot([], [], 'r-', label='Angulo')
        self.output_line, = self.ax_output.plot([], [], 'm-', label='Salida PID')
        
        # Leyendas
        self.ax_system.legend()
        self.ax_pos.legend()
        self.ax_angle.legend()
        self.ax_output.legend()
        
        plt.tight_layout()
        
        # Texto informativo
        self.info_text = self.ax_system.text(0.02, 0.15, '', fontsize=10, 
                                           bbox=dict(boxstyle="round", 
                                                   facecolor="lightblue"))
        
    def animate(self, frame):
        """Funcion de animacion"""
        # Actualiza el simulador
        self.simulator.update_system()
        
        # Actualiza la visualizacion del sistema
        beam_length = self.simulator.L
        angle = self.simulator.beam_angle
        
        # Coordenadas de la barra
        beam_x = [0, beam_length * np.cos(angle)]
        beam_y = [0, beam_length * np.sin(angle)]
        
        # Posicion de la bola
        ball_x = self.simulator.ball_position * np.cos(angle)
        ball_y = self.simulator.ball_position * np.sin(angle)
        
        self.beam_line.set_data(beam_x, beam_y)
        self.ball_point.set_data([ball_x], [ball_y])
        
        # Actualiza las graficas
        if len(self.simulator.time_history) > 1:
            # Mantiene los ultimos 500 puntos
            max_points = 500
            start_idx = max(0, len(self.simulator.time_history) - max_points)
            
            time_data = self.simulator.time_history[start_idx:]
            pos_data = self.simulator.position_history[start_idx:]
            setpoint_data = self.simulator.setpoint_history[start_idx:]
            angle_data = self.simulator.angle_history[start_idx:]
            output_data = self.simulator.output_history[start_idx:]
            
            # Actualiza grafica de posicion
            self.pos_line.set_data(time_data, pos_data)
            self.setpoint_pos_line.set_data(time_data, setpoint_data)
            self.ax_pos.relim()
            self.ax_pos.autoscale_view()
            
            # Actualiza grafica de angulo
            self.angle_line.set_data(time_data, angle_data)
            self.ax_angle.relim()
            self.ax_angle.autoscale_view()
            
            # Actualiza grafica de salida PID
            self.output_line.set_data(time_data, output_data)
            self.ax_output.relim()
            self.ax_output.autoscale_view()
        
        # Actualiza texto informativo
        distance_cm = self.simulator.ball_position * 100
        info = f'Posicion: {distance_cm:.1f} cm\n'
        info += f'Setpoint: {self.simulator.setpoint*100:.1f} cm\n'
        info += f'Error: {self.simulator.error*100:.1f} cm\n'
        info += f'Angulo: {np.degrees(self.simulator.beam_angle):.1f} grados\n'
        info += f'Kp={self.simulator.Kp}, Ki={self.simulator.Ki}, Kd={self.simulator.Kd}'
        
        self.info_text.set_text(info)
        
        return (self.beam_line, self.ball_point, self.pos_line, 
                self.setpoint_pos_line, self.angle_line, self.output_line, self.info_text)
    
    def run(self):
        """Ejecuta la simulacion"""
        self.simulator.last_time = time.time()
        
        # Crea la animacion
        ani = animation.FuncAnimation(self.fig, self.animate, interval=50, 
                                    blit=False, cache_frame_data=False)
        
        plt.show()
        return ani

def main():
    print("Simulador del Sistema Bola y Barra con Control PID")
    print("Basado en codigo de Arduino")
    print("Parametros iniciales:")
    print("  Setpoint: 12 cm")
    print("  PID: Kp=15.0, Ki=0.1, Kd=8.0")
    print("  Posicion inicial: 30 cm")
    print("Iniciando simulacion...")
    
    # Crea y ejecuta el simulador
    gui = SimulatorGUI()
    animation_obj = gui.run()
    
    return animation_obj

if __name__ == "__main__":
    ani = main()