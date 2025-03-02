#!/bin/bash
# ========== Experiment Function ==========
function run_all_combinations() {
  for eval_planner in "${eval_planners[@]}"; do

    for evaluator in "${evaluators[@]}"; do

      experiment_date=$(date '+%Y-%m-%d-%H-%M-%S')

      for planner in "${planners[@]}"; do

        # Get the planner's current Git commit ID
        if [[ $planner == "ipp_2d" ]]; then
          git_dir=${ROS_WS_DIR}/src/scouting-ipp/.git
        else
          echo "Selected planner is not supported: ${planner}"
          return 1
        fi
        planner_commit_id=$(git --git-dir=${git_dir} rev-parse --short --verify HEAD)

        for map in "${maps[@]}"; do

          map_path="${map_dir}/${map}.png"
          map_vis_path="${map_dir}/visualization/${map}_vis.png"
          map_config_path="${map_dir}/cfg/${map}.yaml"
          bounds_path="${map_dir}/cfg/${map}_bounds.yaml"

          # Set the params for the current environment and planner
          launch_file="run_${planner}"
          planner_config="planners/example_config.yaml"
          evaluator_config="evaluators/${evaluator}.yaml"

          if [[ $planner == "ipp_2d" ]]; then
            planner_config_path="${ROS_WS_DIR}/src/scouting-ipp/advanced_planner_ros/cfg/${planner_config}"
            evaluator_config_path="${ROS_WS_DIR}/src/scouting-ipp/advanced_planner_ros/cfg/${evaluator_config}"
          else
            planner_config_path=""
            evaluator_config_path=""
          fi


          batch_name="${experiment_date}_${planner_commit_id}"

          # Set the logging directory
          target_dir="${data_dir}/${planner}/${batch_name}/${map}"

          # Run one batch of experiments with identical settings
          run_experiment_batch
        done 
      done
    done

    experiment_end_date=$(date '+%Y-%m-%d-%H-%M-%S')

    if [ ! -d "$send_notification" ]
    then
      for email_address in "${email_addresses[@]}"; do
        send_email_notification
      done
    fi

  done
}

function run_experiment_batch() {
  echo "Starting experiment series ${launch_file} with evaluator '${evaluator}' and evaluation_planner '${eval_planner}' of ${n_experiments} runs at ${target_dir}!"

  # Run the experiments
  for ((i = 1; i <= n_experiments; i++)); do
    run_dir="${target_dir}/run_${i}"
    # Create dirs
    if [ ! -d "$run_dir" ]; then
      mkdir -p $run_dir
    fi
    if [ ! -d "$record_visualization" ]; then
      mkdir -p "${run_dir}/tmp_bags"
    fi
    if [ ! -d "$record_evaluator_paths" ]; then
      evaluator_paths_dir="${run_dir}/evaluator_paths"
      mkdir -p "$evaluator_paths_dir"
    else
      evaluator_paths_dir=""
    fi
    if [[ $evaluator == "baseline" ]]; then
      is_baseline=true
      trajectory_file_path="${run_dir}/trajectory_file_mav_simulator.csv"
    else
      is_baseline=false
      trajectory_file_path="/tmp/trajectory_file_mav_simulator.csv"
      if [[ $for_visualization == "true" ]]; then #TODO: remove after visualization is done
        trajectory_file_path="${run_dir}/trajectory_file_mav_simulator.csv"
      fi
    fi
    # run experiment
    echo $trajectory_file_path
    echo "roslaunch ros_planner_planexp $launch_file.launch evaluate:=True planner_config:=$planner_config evaluator_config:=$evaluator_config eval_directory:=$run_dir time_limit:=$duration eval_frequency:=$evaluation_frequency record_visualization:=$record_visualization map_path:=$map_path map_config_path:=$map_config_path experiment_config:=$bounds_path paths_path:=$evaluator_paths_dir ignore_mav_obstacles:=$ignore_mav_obstacles abort_on_collision:=$abort_on_collision is_baseline:=$is_baseline trajectory_file_path:=$trajectory_file_path eval_planner:=$eval_planner"
    roslaunch ros_planner_planexp $launch_file.launch evaluate:=True planner_config:=$planner_config evaluator_config:=$evaluator_config eval_directory:=$run_dir time_limit:=$duration eval_frequency:=$evaluation_frequency record_visualization:=$record_visualization map_path:=$map_path map_config_path:=$map_config_path experiment_config:=$bounds_path paths_path:=$evaluator_paths_dir ignore_mav_obstacles:=$ignore_mav_obstacles abort_on_collision:=$abort_on_collision is_baseline:=$is_baseline trajectory_file_path:=$trajectory_file_path eval_planner:=$eval_planner

    if [[ $evaluator == "baseline" ]]; then
      echo "********************************************************************************" #TODO: add eval_planner support here
      echo "python ${ROS_WS_DIR}/src/scouting-ipp/helper_scripts/compute_baseline_performance.py -m ${map_path} -c ${map_config_path} -d ${run_dir} ${trajectory_file_path}"
      python ${ROS_WS_DIR}/src/scouting-ipp/helper_scripts/compute_baseline_performance.py -m ${map_path} -c ${map_config_path} -d ${run_dir} ${trajectory_file_path}
    fi

  done

  eval_dir="${target_dir}/evaluation"
  if [ ! -d "$eval_dir" ]; then
    mkdir -p $eval_dir
  fi

  name_str="${batch_name}_${planner}_${map}"

  if [ ! "$is_baseline" = true ] ; then

    commands_file="${eval_dir}/${name_str}_commands.txt"
    touch ${commands_file}

    echo "********************************************************************************"
    echo "python ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/evaluation.py --dest ${eval_dir} --name ${name_str} --map_config ${map_config_path} --planner_config ${planner_config_path} --planner_bounds_config ${bounds_path} --evaluator_config ${evaluator_config_path} --eval_planner ${eval_planner} ${target_dir}"
    echo "python ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/evaluation.py --dest ${eval_dir} --name ${name_str} --map_config ${map_config_path} --planner_config ${planner_config_path} --planner_bounds_config ${bounds_path} --evaluator_config ${evaluator_config_path} --eval_planner ${eval_planner} ${target_dir}" >> ${commands_file}
    python ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/evaluation.py --dest ${eval_dir} --name ${name_str} --map_config ${map_config_path} --planner_config ${planner_config_path} --planner_bounds_config ${bounds_path} --evaluator_config ${evaluator_config_path}  --eval_planner ${eval_planner} ${target_dir}

    plotting_dir="${eval_dir}/plots"
    if [ ! -d "$plotting_dir" ]; then
      mkdir -p $plotting_dir
    fi

    data_file="${eval_dir}/${name_str}.pickle"

    echo "********************************************************************************"
    echo "python3 ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/plotting.py --dest ${plotting_dir} --map ${map_vis_path} ${data_file}"
    echo "" >> ${commands_file}
    echo "python3 ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/plotting.py --dest ${plotting_dir} --map ${map_vis_path} ${data_file}" >> ${commands_file}
    python3 ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/plotting.py --dest ${plotting_dir} --map ${map_vis_path} ${data_file}

    echo "********************************************************************************"
    echo "python3 ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/performance.py --dest ${eval_dir} ${data_file}"
    echo "" >> ${commands_file}
    echo "python3 ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/performance.py --dest ${eval_dir} ${data_file}" >> ${commands_file}
    python3 ${ROS_WS_DIR}/src/scouting-ipp/data_analysis/performance.py --dest ${eval_dir} ${data_file}

  else
    echo "********************************************************************************"
    echo "python3 ${ROS_WS_DIR}/src/scouting-ipp/helper_scripts/compute_baseline_performance.py -m ${map_path} -c ${map_config_path} -r ${target_dir}"
    python3 ${ROS_WS_DIR}/src/scouting-ipp/helper_scripts/compute_baseline_performance.py -m ${map_path} -c ${map_config_path} -r ${target_dir}
  fi

  echo "********************************************************************************"
  echo "Experiment series ${launch_file} with evaluator '${evaluator}' of ${n_experiments} runs at ${target_dir} finished!"
}

function send_email_notification() {
  mail_file="/tmp/${experiment_date}_mail.txt"
  touch ${mail_file}
  echo "To: ${email_address}
  Subject: Experiment ${experiment_date} finished!
  From: ${smtp_mail}

  Hello,

  your IPP experiment with evaluator '${evaluator}' started at ${experiment_date} is finished now (${experiment_end_date}) and the results of the last batch can be found under:
    ${data_dir}/${planner}/${batch_name}
  Please come visit me and have a great day!

  Cheers,
  Your IPP Experiment Server" > ${mail_file}
  sendmail ${email_address} < ${mail_file}
  echo "Notification mail has been sent to: ${email_address}"
}

# ========== Experiment params: FIXED ==========
# NOTE: The params in this section will be used for all batches.
# Input and output paths
home_dir="/home/${USER}"

map_dir="${home_dir}/data/maps"

data_dir="${home_dir}/data/automated_tests"

exp_env="exp_env"

# Send notification that experiment finished
send_notification=false
declare -a email_addresses=("your.mail@domain.com")
smtp_mail="ipp.mail@domaine.com"


# Experiment runs
n_experiments=10
evaluation_frequency=10  #s
duration=15 #min
ignore_mav_obstacles=true
for_visualization=false


# ========== Experiment params: Varying from batch to batch ==========
# NOTE: One batch will be run for each combination of the params in this section.
# Planners to run
declare -a planners=("ipp_2d")
# Options: "ipp_2d"

# Evaluation planner
declare -a eval_planners=("prm_star")
# Options: "rrt_star" "prm_star"

# Evaluator to run
declare -a evaluators=("rrt_star")
# Options: "baseline" "average_view_cost" "expand_low_cost_area" "expand_reachable_area" "goal_distance" "naive" "rrt_star" "rrt_star_heuristic"

# Map to use
declare -a maps=("map_112" "map_212")

# Evaluation
record_visualization=false
record_evaluator_paths=true
abort_on_collision=false

# ========== Run ==========
# Run one batch of experiments for each planner and drift level combination
run_all_combinations
