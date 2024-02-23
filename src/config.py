# Module used to read the configuration file
import yaml
import os

def parse_settings_file(filename):

    if not os.path.exists(filename):
        print('File does not exist:', filename)
        quit()

    print('Using for calibration settings: ', filename)

    with open(filename) as f:
        settings = yaml.safe_load(f)

    return settings
