import pandas as pd
import math
import numpy as np
class data_manager():
    def __init__(self):
        self.text_to_print=""


    def load_data(self, path):
        self.path=path
        self.name=path.split("/")[-1]
        self.abs_path=path[:-len(path.split("/")[-1])]
        self.df=pd.read_csv(path)

    def apply_heading_correction(self, column_name):
        offset=0
        value_before=0
        update_heading=[]
        update_heading_deg=[]
        for value in self.df[column_name]:
            if value_before>2.0 and value<-2.0 : 
                offset=offset+(math.pi*2)
                value_before=value
            elif value_before<-2.0 and value>2.0 :
                offset=offset-(math.pi*2)
                value_before=value
            else:
                value_before=value
            update_heading.append(value+offset)
            update_heading_deg.append((value+offset)*(180.0/math.pi))
        self.df[f'updated_{column_name}'] = update_heading
        self.df[f'updated_indeg_{column_name}'] = update_heading_deg

    def remove_offset(self, column_name):
        offset=self.df[column_name].iloc[0]
        rmOffSet=[]
        for value in self.df[column_name]:
            rmOffSet.append(value-offset)
        self.df[f'rmOffSet_{column_name}'] = rmOffSet

    def sum(self, column_name_1, column_name_2):
        self.df[f'sum_{column_name_1}_{column_name_2}'] = self.df[column_name_1] + self.df[column_name_2]
        
    def save(self):
        self.df.to_csv(f"{self.abs_path}update_{self.name}", index=False)

    def calculate_distance_moved(self, name, var_x, var_y):
        distances = np.sqrt(np.diff(self.df[var_x])**2 + np.diff(self.df[var_y])**2)
        total_dist=np.sum(distances)
        self.text_to_print+=f"in {name} has moved {total_dist}\n"
        return total_dist
    
    def calculate_error_dist(self, name, x_var, y_var):
        error_X=self.df[x_var].iloc[0]-self.df[x_var].iloc[-1]
        error_y=self.df[y_var].iloc[0]-self.df[y_var].iloc[-1]
        error_xy=math.sqrt(error_X**2+error_y**2)
        self.text_to_print+=f"in {name}: the errors are \n - X : {error_X} \n - Y : {error_y}\n - XY : {error_xy}\n"

    def calculate_dist_along_heading(self, name, x_var, y_var , heading):
        
        self.df['delta_x_local'] = 0.0
        self.df['delta_y_local'] = 0.0
        self.df['sum_delta_x_local'] = 0.0
        self.df['sum_delta_y_local'] = 0.0
        
        for i in range(1, len(self.df)):
            dx = self.df.loc[i, x_var] - self.df.loc[i-1, x_var]
            dy = self.df.loc[i, y_var] - self.df.loc[i-1, y_var]
            theta = np.deg2rad(self.df.loc[i, heading])  # Usa l'angolo **corrente**

            # Applico la rotazione inversa
            dx_local = dx * np.cos(theta) + dy * np.sin(theta)
            dy_local = -dx * np.sin(theta) + dy * np.cos(theta)

            self.df.loc[i, 'delta_x_local'] = dx_local
            self.df.loc[i, 'delta_y_local'] = dy_local
            self.df.loc[i, 'sum_delta_x_local'] = self.df.loc[i-1, 'sum_delta_x_local'] + dx_local
            self.df.loc[i, 'sum_delta_y_local'] = self.df.loc[i-1, 'sum_delta_y_local'] + dy_local

        self.text_to_print+=f"solving differential for {name}\n"
        self.text_to_print+=f" - total long X : {self.df.loc[i-1, 'sum_delta_x_local']}\n"
        self.text_to_print+=f" - total long y : {self.df.loc[i-1, 'sum_delta_y_local']}\n"

    def write_text(self, name):
        with open(f"{self.abs_path}{name}.info", "w") as f:
            f.write(self.text_to_print)
        print(self.text_to_print)
        
        
mix=data_manager()
fileN="000"
mix.load_data(f"/openRMF_ws/src/tesi_code/result/data_odom_{fileN}.csv")
mix.apply_heading_correction("heading")
mix.save()
mix.calculate_distance_moved("wheels", "X (m)","Y (m)")

mix.load_data(f"/openRMF_ws/src/tesi_code/result/data_tf_{fileN}.csv")
mix.apply_heading_correction("heading odom_base")
mix.remove_offset("X (m) map_odom")
mix.remove_offset("Y (m) map_odom")
mix.remove_offset("heading map_odom")
mix.apply_heading_correction("rmOffSet_heading map_odom")
mix.sum("X (m) odom_base","rmOffSet_X (m) map_odom")
mix.sum("Y (m) odom_base","rmOffSet_Y (m) map_odom")
mix.calculate_dist_along_heading("ekf_odom", "X (m) odom_base","Y (m) odom_base", "updated_indeg_heading odom_base")
mix.save()
mix.calculate_distance_moved("ekf_odom", "X (m) odom_base","Y (m) odom_base")
mix.calculate_error_dist("ekf_odom", "X (m) odom_base","Y (m) odom_base")

mix.load_data(f"/openRMF_ws/src/tesi_code/result/data_imu_{fileN}.csv")
mix.apply_heading_correction("heading imu")
mix.remove_offset("updated_indeg_heading imu")
mix.apply_heading_correction("roll imu")
mix.remove_offset("updated_indeg_roll imu")
mix.apply_heading_correction("pitch imu")
mix.remove_offset("updated_indeg_pitch imu")
mix.save()

mix.write_text(fileN)