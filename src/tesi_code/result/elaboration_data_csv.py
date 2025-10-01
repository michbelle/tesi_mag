import pandas as pd
import math
class data_manager():
    def __init__(self):
        pass


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
            if value_before>2.5 and value<-2.5 : 
                offset=offset+(math.pi*2)
                value_before=value
            elif value_before<-2.5 and value>2.5 :
                offset=offset-(math.pi*2)
                value_before=value
            else:
                value_before=value
            update_heading.append(value+offset)
            update_heading_deg.append((value+offset)*(180.0/math.pi))
        self.df[f'updated_{column_name}'] = update_heading
        self.df[f'updated_indeg_{column_name}'] = update_heading_deg

    def remove_map_offset(self, column_name):
        offset=self.df[column_name].iloc[0]
        rmOffSet=[]
        for value in self.df[column_name]:
            rmOffSet.append(value-offset)
        self.df[f'rmOffSet_{column_name}'] = rmOffSet

    def sum(self, column_name_1, column_name_2):
        self.df[f'sum_{column_name_1}_{column_name_2}'] = self.df[column_name_1] + self.df[column_name_2]
        
    def save(self):
        self.df.to_csv(f"{self.abs_path}update_{self.name}", index=False)



mix=data_manager()
fileN="007"
mix.load_data(f"/openRMF_ws/src/tesi_code/result/data_odom_{fileN}.csv")
mix.apply_heading_correction("heading")
mix.save()


mix.load_data(f"/openRMF_ws/src/tesi_code/result/data_tf_{fileN}.csv")
mix.apply_heading_correction("heading odom_base")
mix.remove_map_offset("X (m) map_odom")
mix.remove_map_offset("Y (m) map_odom")
mix.remove_map_offset("heading map_odom")
mix.apply_heading_correction("rmOffSet_heading map_odom")
mix.sum("X (m) odom_base","rmOffSet_X (m) map_odom")
mix.sum("Y (m) odom_base","rmOffSet_Y (m) map_odom")
mix.save()