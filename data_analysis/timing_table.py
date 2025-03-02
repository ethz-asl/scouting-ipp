import copy
import os
import errno
import argparse
import numpy as np
import pandas as pd

def parse_arguments():
    parser = argparse.ArgumentParser(description='Compute timing table for multiple runs, possibly on different maps, with different evaluators.')
    parser.add_argument('--dest', dest='destination_folder', type=str, default=None,
                        help='output folder where plots shall be stored')
    parser.add_argument('--names', dest='names', nargs="*", type=str, default=None,
                        help='')
    parser.add_argument('--files', dest='exp_files', nargs="*", type=str,
                        help='paths to the performance_runs_table.csv files')
    parser.add_argument('--file_name', dest='file_name', type=str, default=None,
                        help='output file name')
    parser.add_argument('--columns', dest='columns', nargs="*", type=str,
                        help='name of the columns to be included in the output')
    args = parser.parse_args()
    return args

def format_mean_std(input):
    (mean, std_dev) = input
    if np.isnan(mean) or np.isnan(std_dev): 
        return "-" 
    return "$" + "{:.0f}".format(mean) + "\pm" + "{:.0f}".format(std_dev) + "$"


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()
    destination_folder = arguments.destination_folder
    experiments = []
    names = []

    use_names = False
    if arguments.names is not None:
        if len(arguments.names) == len(arguments.exp_files):
            use_names = True
        else:
            print("Number of names and number of experiments does not match. Using default names instead!")
            use_names = False
    else:
        use_names = False

    
    if use_names: 
        names = arguments.names
    else: 
        names = arguments.exp_files

    columns = []

    all_group_means = {}

    evaluators = []

    for i, exp_file_path in enumerate(arguments.exp_files):
        name = names[i]
        dataframe = pd.read_csv(exp_file_path)
        dataframe["file_name"] = name
        experiments.append(dataframe)
    
        if len(evaluators) == 0: 
            evaluators = dataframe["Name"].unique()
        elif not (evaluators == dataframe["Name"].unique()).all(): 
            exit(1)

        if len(columns) == 0: 
            columns = dataframe.columns
        elif not (columns == dataframe.columns).all(): 
            exit(1)
        
        data_columns = list(filter(lambda x: x not in ["Name", "Run", "file_name"], columns))
        
        name_col = dataframe["Name"].unique()
        
        grouped = dataframe.groupby("Name")

        for group in name_col: 
            for col in data_columns: 
                all_group_means[(name, group, col)] = np.nanmean(grouped.get_group(group)[col])
    
    total_data = pd.concat(experiments)
    with open(os.path.join(destination_folder, "total_data.csv"), 'w') as f:
        f.write(total_data.to_csv(index=False))

    grouped = total_data.groupby("Name")
    print(total_data)
    all_mean_std = {}
    for evaluator in evaluators: 
        evaluator_data = grouped.get_group(evaluator)
        for col in data_columns:
            col_data = evaluator_data[col].to_numpy()
            accum = np.array([(col_data[i] - all_group_means[(evaluator_data["file_name"].to_list()[i], evaluator, col)])**2 for i in range(len(col_data))])
            std_dev = np.sqrt(np.nanmean(accum))
            mean = np.nanmean(col_data)
            all_mean_std[(evaluator, col)] = (mean, std_dev)
    
    result = pd.DataFrame.from_dict({"Evaluators": evaluators})

    if arguments.columns is None: 
        output_columns = data_columns
    else: 
        use_cols = [col.lower() for col in arguments.columns]
        output_columns = list(filter(lambda col: col.lower() in use_cols , data_columns))
    
    for col in output_columns: 
        result[col] = [all_mean_std[(evaluator, col)] for evaluator in evaluators]

    formater = lambda row: "${:,.0f} \pm {:,.0f}$".format(row[0], row[1])
    formatters = {}
    for col in result.columns[1:]:
        formatters[col]=formater

    with open(os.path.join(destination_folder, "mean_std_table.csv"), 'w') as f:
        f.write(result.set_index(result.columns[1]).transpose().iloc[1:].to_csv(index=False))

    
    column_format = "c|" + "c"*(len(result.index))

    latex_table = result.set_index(result.columns[0]).apply(formatters).transpose().to_latex(formatters=formatters, index=True, escape=False, na_rep="", column_format=column_format)

    replacements = {"IPP Cost To First Path": "\\tfirst",
                    "IPP Cost To Best Path 1.0\%":"\\topt",
                    "IPP Cost To Termination":"\\tterm",
                    "Time To First Path": "\\tfirst",
                    "Time To Best Path 1.0\%":"\\topt",
                    "Time To Termination":"\\tterm",
                    "Evaluators": "Metric",
                    "$nan \pm nan$": "N/A"}
    
    for key in replacements:
        latex_table = latex_table.replace(key, replacements[key])

    with open(os.path.join(destination_folder, "mean_std_table.tex"), 'w') as f:
        f.write(latex_table)
        print(latex_table)


        


    
    

        
