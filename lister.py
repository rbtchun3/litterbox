#!/usr/bin/env python3

import yaml

# Specify the file path to read
file_path = "./candidate.txt"

def get_second_index(string, char):
    """Returns the second index of the specified character from the back of the string."""
    try:
        last_index = string.rindex(char)  # Get the last index of the character
        second_last_index = string.rindex(char, 0, last_index)  # Get the second last index of the character
        return second_last_index
    except ValueError:
        return -1
bags = []
# Read the contents of the file into a list
with open(file_path, "r") as f:
    while True:
        bag_path = f.readline()
        # example line is /hdd7/nursery_data_collection/20221208_104831_archive
        # or /hdd5/burro_bags_3/6017/6017/20220121_144529_archive/20220121_144529_archive.bag
        if not bag_path:
            break
        bag_path = bag_path.strip(' \n')
        if bag_path[-3:] == 'bag':
            bag_path_reduced = bag_path[:bag_path.rindex('/')]
            bag_dir = bag_path_reduced[:bag_path_reduced.rindex('/')]
            bag_name = bag_path_reduced[bag_path_reduced.rindex('/'):]
        else:
            bag_dir = bag_path[:bag_path.rindex('/')]
            bag_name = bag_path[bag_path.rindex('/'):]

        bag_config = {}
        bag_config['bagdir'] = bag_dir
        bag_config['bagfile'] = bag_name
        bag_config['unit_test_thresholds'] = {}
        bag_config['unit_test_thresholds']['RPE_2m_max'] = bag_config['unit_test_thresholds']['RPE_2m_median'] = bag_config['unit_test_thresholds']['RPE_20m_max'] = bag_config['unit_test_thresholds']['RPE_20m_median'] = -1
        bag_config['unit_test_thresholds']['ATE_max'] = bag_config['unit_test_thresholds']['ATE_rmse'] = bag_config['unit_test_thresholds']['error_variation_max'] = -1



        # print(f"bag_path: {bag_path} bagdir: {bag_dir} and bag_name: {bag_name}")
        bags.append(bag_config)
# # Strip newline characters from each line
# lines = [line.strip() for line in lines]

# # Convert the list to YAML format
yaml_data = yaml.dump(bags, default_flow_style=False)

# # Output the YAML data
print(yaml_data)



# - bagdir: "/hdd5/burro_bags_3/6015"
#   bagfile: "20221121_232505_base"
#   unit_test_thresholds:  # Derived by taking 15 test runs and adding 10% of the variance to the max value for each
#     RPE_2m_max: 0.28
#     RPE_2m_median: 0.14
#     RPE_20m_max: 2.78
#     RPE_20m_median: 1.35
#     ATE_max: 8.45
#     ATE_rmse: 4.34
#     error_variation_max : 0.72
# - bagdir: "/hdd7/nursery_data_collection"
#   bagfile: "20221208_124300_archive"
#   unit_test_thresholds:
#     RPE_2m_max: 0.92
#     RPE_2m_median: 0.08
#     RPE_20m_max: 5.06
#     RPE_20m_median: 0.95
#     ATE_max: 11.89
#     ATE_rmse: 5.19
#     error_variation_max : 0.86