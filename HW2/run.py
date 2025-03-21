import argparse
from MapEnvironment import MapEnvironment
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
from RCSPlanner import RCSPlanner

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map1.json', help='Json file name containing all map information')
    parser.add_argument('-planner', '--planner', type=str, default='rrt', help='The planner to run. Choose from [astar, rrt, rrtstar]')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1', help='edge extension mode for RRT and RRTStar')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.05, help='probability to draw goal vertex for RRT and RRTStar')
    parser.add_argument('-k', '--k', type=int, default=1, help='number of nearest neighbours for RRTStar')
    args = parser.parse_args()

    # prepare the map
    planning_env = MapEnvironment(json_file=args.map)

    # setup the planner


    # execute plan
    total_cost = 0
    total_time = 0
    for i in range(10):
        if args.planner == 'rcs':
            planner = RCSPlanner(planning_env=planning_env)
        elif args.planner == 'rrt':
            planner = RRTPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob)
        elif args.planner == 'rrtstar':
            planner = RRTStarPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob,
                                     k=args.k)
        else:
            raise ValueError('Unknown planner option: %s' % args.planner);
        plan, cost, time = planner.plan()
        total_cost += cost
        total_time += time

    # visualize the final path with edges or states according to the requested planner.
        if args.planner == 'rcs':
            planner.planning_env.visualize_map(plan=plan, expanded_nodes=planner.get_expanded_nodes())
        else:
            planner.planning_env.visualize_map(plan=plan, tree_edges=planner.tree.get_edges_as_states())
    print(f'Cost: {total_cost / 10}, Time: {total_time / 10}')