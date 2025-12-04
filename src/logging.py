#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import csv
import json

class logger:
    def __init__(self, filename = "output"):
        self.has_written_csv_header = False
        self.filename = filename
        self.iter = 1

    def _flatten_dict(self, data, flattened_data={}, topkey="", sep=""):
        """Flattens a dictionary with sub-dictionaries into a single dictionary."""
        for key, value in data.items():
            if isinstance(value, dict):
                flattened_data = self._flatten_dict(value, flattened_data, key, "_")
            else:
                flattened_data[f"{topkey}{sep}{key}"] = value

        return flattened_data

    def write_dict_to_csv(self, data):
        """Writes flattened dictionary to a CSV file."""
        
        # Get filename with extension
        filename = self.filename + ".csv"

        # Get data to write
        flattened_data = self._flatten_dict(data)
        
        # Write
        with open(filename, mode="w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=flattened_data.keys())
            
            if not self.has_written_csv_header:
                # Write header (keys from first dictionary entry)
                writer.writerow(flattened_data.keys())
                self.has_written_csv_header = True
            
            # Write rows (values)
            writer.writerow(flattened_data.values())

    def write_dict_to_json(self, data):
        """Writes dict to a JSON file"""
        # Get filename with extension
        filename = self.filename + ".json"

        # Get iterable
        data = {f"iter_{self.iter}": data}

        if self.iter > 1:
            with open(filename, "r") as file:
                write_data = json.load(file)

                write_data.update(data)

        else:
            write_data = data

        # Write
        mode = "w" if self.iter > 1 else "w"
        with open(filename, mode=mode) as file:
            json.dump(write_data, file, indent=4)  # Pretty formatting for readability

        self.iter += 1