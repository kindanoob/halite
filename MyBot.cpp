#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <ctime>
#include <time.h>
#include <set>
#include <fstream>
#include <cmath>
#include <map>
#include <chrono>

#include "hlt.hpp"
#include "networking.hpp"



const int CAP_BOUND = 265;
const float EPSILON = 0.01;

unsigned char get_nearest_direction(const hlt::Location &start_location, const hlt::Site &start_site,
                                    hlt::GameMap &present_map, unsigned char width, unsigned char height, unsigned char my_id)
{
    unsigned char max_dist = ((width < height) ? (width) : (height)) / 2;
    unsigned char best_dir = NORTH;
    for(int i = 0; i < 4; i++)
    {
        unsigned char dist = 0;
        unsigned char curr_direction = CARDINALS[i];
        hlt::Location curr_location = start_location;
        hlt::Site curr_site = start_site;
        while((curr_site.owner == my_id) && (dist < max_dist))
        {
            curr_location = present_map.getLocation(curr_location, curr_direction);
            curr_site = present_map.getSite(curr_location);
            dist++;
        }
        if(dist < max_dist)
        {
            max_dist = dist;
            best_dir = curr_direction;
        }
    }
    return best_dir;
}

float compute_force (hlt::Location original_location, hlt::Site site, hlt::Location location, hlt::GameMap present_map, float dist, int myArea, int my_id)
{
    float force = 0.0;
    if(site.owner == 0)
    {
        float production = (float) site.production;
	    float strength = (float) site.strength;
        int num_neighbors = 0;
        float neighbor_force = 0.0;

        for(int i = 0; i < 4; i++)
        {
            unsigned char dir = CARDINALS[i];
            hlt::Location neighbor_loc = present_map.getLocation(location, dir);
            hlt::Site neighbor_site = present_map.getSite(neighbor_loc);
            if(neighbor_site.owner != my_id)
            {
                float neighbor_production = (float) neighbor_site.production;
                float neighbor_strength = (float) neighbor_site.strength;
                float neighbor_dist = present_map.getDistance(original_location, neighbor_loc);
                float neighbor_val = (neighbor_strength ? (neighbor_production / neighbor_strength) : neighbor_production) / (neighbor_dist * neighbor_dist * neighbor_dist * 1);
                neighbor_force += neighbor_val;
                num_neighbors++;
            }
        }
        force = (((strength ? (production / strength) : production) / (dist * dist * dist * 1)) + neighbor_force) / (num_neighbors + 1);

    }
    else//that is, if owner is not in (0, myID)
    {
        force = site.production / (dist * dist * dist) ;
        for(int i = 0; i < 4; i++)
        {
            unsigned char dir = CARDINALS[i];
            hlt::Location neighbor_loc = present_map.getLocation(location, dir);
            hlt::Site neighbor_site = present_map.getSite(neighbor_loc);
            if((neighbor_site.owner != 0) && (neighbor_site.owner != my_id))
            {
                float neighbor_dist = present_map.getDistance(neighbor_loc, original_location);
                float strength = (float) neighbor_site.strength / (neighbor_dist * neighbor_dist * neighbor_dist);
                force += strength;
            }
        }
    }
	return force;
}

float heuristic(const hlt::Site &curr_site, const hlt::Location &curr_location, hlt::GameMap &present_map, const unsigned char &my_id)
{
    if((curr_site.owner == 0) && (curr_site.strength > 0))
    {
        return static_cast<float>(curr_site.production) / curr_site.strength;
    }
    else
    {
        float strength;
        if(curr_site.owner == 0)
        {
            strength = curr_site.production;
        }
        else
        {
            strength = 0.0;
        }

        for(int i = 0; i < 4; i++)
        {
            int direction = CARDINALS[i];
            hlt::Site site = present_map.getSite(curr_location, direction);
            if((site.owner != my_id) && (site.owner != 0))
            {
                strength += site.strength;
            }
        }
        return strength;
    }
}

bool is_on_border(const hlt::Location &location, hlt::GameMap present_map, const unsigned char my_id)
{
    for(int i = 0; i < 4; i++)
    {
        unsigned char direction = CARDINALS[i];
        hlt::Site curr_site = present_map.getSite(location, direction);
        if(curr_site.owner != my_id)
        {
            return true;
        }
    }
    return false;
}

hlt::Location get_best_target_on_border_location(hlt::GameMap &present_map, unsigned char my_id)
{
    float max_val = -1.0;
    hlt::Site best_site;
    hlt::Location best_loc;
    for(unsigned short a = 0; a < present_map.height; a++)
    {
        for(unsigned short b = 0; b < present_map.width; b++)
        {
            hlt::Location curr_location = {b, a};
            hlt::Site curr_site = present_map.getSite(curr_location);
            if (curr_site.owner == my_id)
            {
                for(int i = 0; i < 4; i++)
                {
                    unsigned char neighbor_direction = CARDINALS[i];
                    hlt::Location neighbor_location = present_map.getLocation(curr_location, neighbor_direction);
                    hlt::Site neighbor_site = present_map.getSite(neighbor_location);
                    if(neighbor_site.owner != my_id)
                    {
                        float curr_val = heuristic(neighbor_site, neighbor_location, present_map, my_id);
                        if(curr_val > max_val)
                        {
                            max_val = curr_val;
                            best_site = neighbor_site;
                            best_loc = neighbor_location;
                        }
                    }

                }
            }
        }
    }
    return best_loc;
}


unsigned char get_best_target_on_border_direction(hlt::Location &start_location, hlt::Location &goal_location,
                    hlt::GameMap &present_map, unsigned char my_id, std::map<hlt::Location, int> &reserved_own,
                    std::map<hlt::Location, int> &reserved_enemy, std::ofstream &output_file)
{
    #ifdef DEBUG
        output_file << "entered get_best_target_on_border_direction with start" << start_location.x << ", " << start_location.y << ", and goal " << goal_location.x << ", " << goal_location.y << std::endl;
    #endif // DEBUG
    hlt::Site start_site = present_map.getSite(start_location);
    hlt::Site goal_site = present_map.getSite(goal_location);
    float dist_start_goal = present_map.getDistance(start_location, goal_location);
    #ifdef DEBUG
        output_file << "dist_start_goal: " << dist_start_goal << std::endl;
    #endif // DEBUG


    if((fabs(dist_start_goal - 1.0) < EPSILON))
    {
        if(start_site.strength <= goal_site.strength)
        {
            #ifdef DEBUG
                output_file << "near target, stand still" << std::endl;
            #endif // DEBUG
            return STILL;
        }
        else
        {
            #ifdef DEBUG
                output_file << "near target, capture" << std::endl;
                output_file << "goal location: " << goal_location.x << ", " << goal_location.y << std::endl;
            #endif // DEBUG
            for(int i = 0; i < 4; i++)
            {
                unsigned char curr_dir = CARDINALS[i];
                hlt::Location curr_location = present_map.getLocation(start_location, curr_dir);
                #ifdef DEBUG
                    output_file << "curr location: " << curr_location.x << ", " << curr_location.y << std::endl;
                #endif // DEBUG
                if((goal_location.x == curr_location.x) && (goal_location.y == curr_location.y) &&
                   (reserved_enemy[goal_location] + start_site.strength < 255))
                {
                    #ifdef DEBUG
                        output_file << "return direction: " << curr_dir << std::endl;
                    #endif // DEBUG
                    return curr_dir;
                }
            }
            return STILL;//if can't capture the goal due to cap restrictions
        }

    }
    hlt::Location best_location = start_location;
    hlt::Site best_site = start_site;
    unsigned char best_direction = STILL;
    float min_distance = dist_start_goal;
    for(int i = 0; i < 4; i++)
    {
        unsigned char curr_direction = CARDINALS[i];
        hlt::Location curr_location = present_map.getLocation(start_location, curr_direction);
        hlt::Site curr_site = present_map.getSite(curr_location);

        if((curr_site.owner == my_id) and is_on_border(curr_location, present_map, my_id))
        {

            if(reserved_own[curr_location] + start_site.strength < 255)
            {
                float curr_distance = present_map.getDistance(curr_location, goal_location);
                #ifdef DEBUG
                    output_file << "curr_distance: " << curr_distance << std::endl;
                #endif // DEBUG
                if(curr_distance < dist_start_goal)
                {
                    min_distance = curr_distance;
                    best_location = curr_location;
                    best_site = present_map.getSite(best_location);
                    best_direction = curr_direction;
                }
            }
        }

    }

    if(best_direction != STILL)
    {
        reserved_own[start_location] = 0;

        if(!reserved_own.count(best_location))
        {
            reserved_own[best_location] = best_site.strength + start_site.strength;
        }
        else
        {
            reserved_own[best_location] += start_site.strength;
        }
    }
    #ifdef DEBUG
        output_file << "best direction: " << static_cast<unsigned>(best_direction) << std::endl;
    #endif // DEBUG
    #ifdef DEBUG
        output_file << "best location: " << best_location.x << ", " << best_location.y << std::endl;
    #endif // DEBUG
    #ifdef DEBUG
        output_file << "reserved_own[best_location]: " << static_cast<unsigned>(reserved_own[best_location]) <<
            ", reserved_own[start_location]: " << static_cast<unsigned>(reserved_own[start_location]) << std::endl;
    #endif // DEBUG


    return best_direction;
}



int main ()
{
	std::ofstream output_file;
    output_file.open("output.txt");
	srand (time (NULL));

	unsigned char myID;
	hlt::GameMap currMap;
	std::set<hlt::Move> moveList;
	getInit (myID, currMap);
	sendInit ("my_c++_bot_v27_test");

	int tick = 0;
	std::map<hlt::Move, bool> prevMap;
	std::map<hlt::Location, int> reserved_own;
	std::map<hlt::Location, int> reserved_enemy;
	while(true)
	{
	    #ifdef DEBUG_TIME
            output_file << "=============================== " << tick << " =====================================" << std::endl;
        #endif // DEBUG
		moveList.clear ();
		getFrame (currMap);
        std::chrono::duration<double, std::milli> think_time = std::chrono::milliseconds(0);
		hlt::Location best_target_on_border_location = get_best_target_on_border_location(currMap, myID);
		#ifdef DEBUG
		    output_file << "best_target_on_border_location: " << best_target_on_border_location.x << ", " << best_target_on_border_location.y << std::endl;
		#endif // DEBUG


		std::set<hlt::Location> border;
		int myArea = 0;
		for (unsigned short a = 0; a < currMap.height; a ++)
		{
			for (unsigned short b = 0; b < currMap.width; b ++)
			{

			    hlt::Site curr_site = currMap.getSite ({b, a}, STILL);
				if (curr_site.owner == myID)
                {
                    myArea ++;
					reserved_own[{b, a}] = curr_site.strength;
					#ifdef DEBUG
					    output_file << "set reserved_own[" << b << ", " << a << "] = " << static_cast<unsigned>(curr_site.strength) << std::endl;
					#endif // DEBUG
                }
				else
				{
					for (unsigned char i = 1; i < 5; i ++)
						if (currMap.getSite ({b, a}, i).owner == myID)
						{
							border.insert ({b, a});
							break;
						}
                    reserved_enemy[{b, a}] = 0;
				}
			}
		}

		for (unsigned short a = 0; a < currMap.height; a++)
		{
			for (unsigned short b = 0; b < currMap.width; b++)
			{
			    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
			    hlt::Location location = {b, a};
			    hlt::Site site = currMap.getSite (location, STILL);
			    unsigned char direction;
				if (site.owner == myID)
				{

				    #ifdef DEBUG
				        output_file << "*************************************CURRENT SQUARE: " << b << ", " << a << std::endl;
				        output_file << "think_time: " << think_time.count() << std::endl;
				    #endif // DEBUG

				    float force_x = 0.0;
                    float force_y = 0.0;
                    bool move_found = false;
				    if(!is_on_border(location, currMap, myID))
                    {
                        if(think_time > std::chrono::milliseconds(900))
                        {
                            #ifdef DEBUG
                                output_file << "IS NOT ON BORDER" << std::endl;
                                output_file << "think_time: " << think_time.count() << std::endl;
                            #endif // DEBUG
                            direction = get_nearest_direction(location, site, currMap, currMap.width, currMap.height, myID);
                            reserved_own[currMap.getLocation(location, direction)] += site.strength;
                            reserved_own[location] -= site.strength;
                            move_found = true;
                        }
                        else
                        {
                            #ifdef DEBUG
                                output_file << "IS NOT ON BORDER" << std::endl;
                                output_file << "site strength: " << static_cast<unsigned>(site.strength) << std::endl;
                            #endif // DEBUG
                            for (std::set<hlt::Location>::iterator it = border.begin (); it != border.end (); it++)
                            {
                                hlt::Site enemy_site = currMap.getSite (*it, STILL);
                                hlt::Location enemy_location = *it;
                                if(true)
                                {
                                    float dist = currMap.getDistance(location, enemy_location);
                                    float angle = currMap.getAngle(location, enemy_location);
                                    float force = compute_force(location, enemy_site, enemy_location, currMap, dist, myArea, myID);
                                    force_x += force * cos(angle);
                                    force_y += force * sin(angle);
                                }
                            }
                            direction = STILL;

                            if ((site.strength + reserved_own[currMap.getLocation(location, EAST)] < CAP_BOUND) && force_x + force_y > 0 && force_x - force_y > 0)
                            {
                                direction = EAST;
                                #ifdef DEBUG
                                    output_file << "info east: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                    output_file << "info east before: reserved_own[" << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << "] = " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                                #endif // DEBUG
                                reserved_own[location] -= site.strength;
                                reserved_own[currMap.getLocation(location, direction)] += site.strength;
                                move_found = true;
                                #ifdef DEBUG
                                    output_file << "info east: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                    output_file << "info east after: reserved_own[" << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << "] = " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                                #endif // DEBUG
                            }
                            else if ((site.strength + reserved_own[currMap.getLocation(location, SOUTH)] < CAP_BOUND) && force_x + force_y > 0 && force_x - force_y < 0)
                            {
                                direction = SOUTH;
                                #ifdef DEBUG
                                    output_file << "info south: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                    output_file << "info south before: reserved_own[" << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << "] = " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                                #endif // DEBUG
                                reserved_own[location] -= site.strength;
                                reserved_own[currMap.getLocation(location, direction)] += site.strength;
                                move_found = true;
                                #ifdef DEBUG
                                    output_file << "info south: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                    output_file << "info south after: reserved_own[" << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << "] = " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                                #endif // DEBUG
                            }
                            else if ((site.strength + reserved_own[currMap.getLocation({b, a}, WEST)] < CAP_BOUND) && force_x + force_y < 0 && force_x - force_y < 0)
                            {
                                direction = WEST;
                                #ifdef DEBUG
                                    output_file << "info west: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                    output_file << "info west before: reserved_own[" << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << "] = " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                                #endif // DEBUG
                                reserved_own[location] -= site.strength;
                                reserved_own[currMap.getLocation(location, direction)] += site.strength;
                                move_found = true;
                                #ifdef DEBUG
                                    output_file << "info west: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                    output_file << "info west after: reserved_own[" << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << "] = " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                                #endif // DEBUG
                            }
                            else if ((site.strength + reserved_own[currMap.getLocation({b, a}, NORTH)] < CAP_BOUND) && force_x + force_y < 0 && force_x - force_y > 0)
                            {
                                direction = NORTH;
                                #ifdef DEBUG
                                    output_file << "info north: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                    output_file << "info north before: reserved_own[" << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << "] = " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                                #endif // DEBUG
                                reserved_own[location] -= site.strength;
                                reserved_own[currMap.getLocation(location, direction)] += site.strength;
                                move_found = true;
                                #ifdef DEBUG
                                    output_file << "info north: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                    output_file << "info north after: reserved_own[" << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << "] = " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                                #endif // DEBUG
                            }
                            #ifdef DEBUG
                                output_file << "found direction: " << static_cast<unsigned>(direction) << std::endl;
                                output_file << "location: " << currMap.getLocation(location, direction).x << ", " << currMap.getLocation(location, direction).y << std::endl;
                                output_file << "info after: " << static_cast<unsigned>(reserved_own.count(currMap.getLocation(location, direction))) << std::endl;
                                output_file << "info after: " << static_cast<unsigned>(reserved_own[currMap.getLocation(location, direction)]) << std::endl;
                            #endif // DEBUG
                        }
                        if(site.strength < 5 * site.production)
                        {
                            #ifdef DEBUG
                                output_file << "too weak, stand still" << std::endl;
                            #endif // DEBUG
                            if(move_found)
                            {
                                reserved_own[currMap.getLocation(location, direction)] -= site.strength;
                                reserved_own[location] += site.strength;
                            }
                            direction = STILL;
                            move_found = true;

                        }
                        else if(!move_found)
                        {
                            #ifdef DEBUG
                                output_file << "move not found" << std::endl;
                            #endif // DEBUG
                        }
                    }

                    else//that is if our square is on border
                    {
                        #ifdef DEBUG
                            output_file << "IS ON BORDER" << std::endl;
                        #endif // DEBUG
                        bool move_found = false;
                        double max_val = -1.0;
                        direction = 123;
                        hlt::Site best_site;
                        hlt::Location best_location;
                        #ifdef DEBUG
                            output_file << "looking for the best target" << std::endl;
                        #endif // DEBUG
                        for(int i = 0; i < 4; i++)
                        {
                            unsigned char curr_direction = CARDINALS[i];
                            hlt::Location curr_location = currMap.getLocation(location, curr_direction);
                            hlt::Site curr_site = currMap.getSite(curr_location);
                            if(curr_site.owner != myID)
                            {
                                float curr_val = heuristic(curr_site, curr_location, currMap, myID);
                                #ifdef DEBUG
                                    output_file << "cardinals index: " << i << std::endl;
                                    output_file << "curr location: " << curr_location.x << ", " << curr_location.y << std::endl;
                                    output_file << "curr val: " << curr_val << ", max val: " << max_val << std::endl;
                                #endif // DEBUG
                                if((curr_val > max_val) && (reserved_enemy[curr_location] + site.strength < CAP_BOUND))
                                {
                                    max_val = curr_val;
                                    direction = curr_direction;
                                    best_site = curr_site;
                                    best_location = curr_location;
                                    #ifdef DEBUG
                                        output_file << "best direction is now " << static_cast<unsigned>(direction) << std::endl;
                                    #endif // DEBUG
                                }
                            }

                        }
                        #ifdef DEBUG
                            output_file << "info before: " << static_cast<unsigned>(site.strength) << ", " << static_cast<unsigned>(best_site.strength)
                            << ", " << static_cast<unsigned>(reserved_enemy[best_location]) << ", " <<
                            reserved_enemy.count(best_location) << std::endl;
                        #endif // DEBUG
                        if((site.strength > best_site.strength) && (reserved_enemy[best_location] + site.strength < CAP_BOUND))
                        {
                            #ifdef DEBUG
                                output_file << "capture best target, dir: " << static_cast<unsigned>(direction) << std::endl;
                            #endif // DEBUG
                            reserved_enemy[best_location] = (reserved_enemy.count(best_location) == 0) ? (reserved_enemy[best_location] = site.strength) : (reserved_enemy[best_location] += site.strength);
                            reserved_own[location] = 0;
                            move_found = true;
                        }
                        else if(site.strength < 5 * site.production)
                        {
                            #ifdef DEBUG
                                output_file << "too weak, stand still" << std::endl;
                            #endif // DEBUG
                            direction = STILL;
                            move_found = true;
                        }
                        if(!move_found)
                        {
                            #ifdef DEBUG
                                output_file << "move not found" << std::endl;
                            #endif // DEBUG
                            direction = get_best_target_on_border_direction(location, best_target_on_border_location, currMap,
                                                   myID, reserved_own, reserved_enemy, output_file);
                        }
                        
                    }


					hlt::Move move = {{b, a}, direction};
					moveList.insert (move);
				}
				std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
				think_time += (end - start);
			}
		}
		tick++;
		sendFrame (moveList);
	}
	output_file.close();

	return 0;
}
