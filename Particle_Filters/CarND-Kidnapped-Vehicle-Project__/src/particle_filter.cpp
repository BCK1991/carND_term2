/*
* particle_filter.cpp
*
*  Created on: Dec 12, 2016
*      Author: Tiffany Huang
*/

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

default_random_engine DRE; // A random number engine class that generates pseudo-random numbers.

/* This function takes the input GPS position and initial heading estimate and an array of uncertainities for this measurement*/
void ParticleFilter::init(double x, double y, double theta, double std[]) {

	ParticleFilter::num_particles = NUM_PARTICLES;

	//Add random Gaussian noise to each particle
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	//initialize all particles with random numbers 
	for (int i = 0; i < num_particles; i++){
		Particle _Sample;
		_Sample.x = dist_x(DRE);
		_Sample.y = dist_y(DRE);
		_Sample.theta = dist_theta(DRE);
		_Sample.weight = 1.0;

		particles.push_back(_Sample);
		ParticleFilter::weights.push_back(1.0);
	}

	ParticleFilter::is_initialized = true;

}
/* This function takes the input amount of time between the time steps, the velocity and the yaw rate measurement uncertainity, and the current velocity and yaw rate measurements */
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	for (int i = 0; i < num_particles; i++)
	{
		normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);

		double theta_noisy = dist_theta(DRE);
		double x_noisy = dist_x(DRE);
		double y_noisy = dist_y(DRE);
		
		//Add measurements to each particle and add random Gaussian noise.
		if (yaw_rate != 0)
		{
			particles[i].x = x_noisy + ((velocity / yaw_rate)*
				(sin(theta_noisy + (yaw_rate * delta_t)) - sin(theta_noisy)));

			particles[i].y = y_noisy + ((velocity / yaw_rate)*
				(cos(theta_noisy) - cos(theta_noisy + (yaw_rate*delta_t))));
		}
		else
		{
			particles[i].x = x_noisy + (velocity * cos(theta_noisy) * delta_t);
			particles[i].y = y_noisy + (velocity * sin(theta_noisy) * delta_t);
		}
		particles[i].theta = theta_noisy + yaw_rate * delta_t;
		//Reset
		dist_x.reset();
		dist_y.reset();
		dist_theta.reset();
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//Replaced internally in the updateWeights function.
	return;
}

/* This function takes the input range of the sensor, the landmark measurement uncertainities, a vector of landmark measurements as input */
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
	const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	double delta_x, delta_y, diff;
	

	for (int i = 0; i < num_particles; i++)
	{
		particles[i].data_associations = observations;
		vector<LandmarkObs> temp;

		for (int j = 0; j < particles[i].data_associations.size(); j++)
		{
			double range = sqrt(pow(observations[j].x, 2) + pow(observations[j].y, 2));
			if (range < sensor_range)
			{
				particles[i].data_associations[j].x = (observations[j].x * cos(particles[i].theta)) - (observations[j].y * sin(particles[i].theta)) + particles[i].x;

				particles[i].data_associations[j].y = (observations[j].x * sin(particles[i].theta)) + (observations[j].y * cos(particles[i].theta)) + particles[i].y;

				temp.push_back(particles[i].data_associations[j]);
			}
		}
		particles[i].data_associations = temp;

		// particles::associations function is implemented here
		for (int j = 0; j < particles[i].data_associations.size(); j++)
		{
			//assign a big number to min_diff as starting value
			double min_diff = 10000;
			for (int k = 0; k < map_landmarks.landmark_list.size(); k++)
			{
				
				delta_x = particles[i].data_associations[j].x - map_landmarks.landmark_list[k].x_f;
				delta_y = particles[i].data_associations[j].y - map_landmarks.landmark_list[k].y_f;
				diff = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

				if (diff < min_diff)
				{
					min_diff = diff;
					particles[i].data_associations[j].id = map_landmarks.landmark_list[k].id_i;
				}
			}
		}

		double normalization = 0.5 / (M_PI * std_landmark[0] * std_landmark[1]);
		particles[i].weight = 1.0;
		//multi-variant gaussian probability density function.
		for (int j = 0; j < particles[i].data_associations.size(); j++)
		{
			Map::single_landmark_s landmark = map_landmarks.landmark_list[particles[i].data_associations[j].id - 1];
			double diff_x = particles[i].data_associations[j].x - landmark.x_f;
			double calculated_X = pow(diff_x, 2) / (2 * pow(std_landmark[0], 2));
			double diff_y = particles[i].data_associations[j].y - landmark.y_f;
			double calculated_Y = pow(diff_y, 2) / (2 * pow(std_landmark[1], 2));
			double weight_temp = exp(-(calculated_X + calculated_Y)) * normalization;

			particles[i].weight *= weight_temp;
		}

		ParticleFilter::weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// Using C++ std's 'discrete_distribution' function here to update the particle's posterior.

	discrete_distribution<> d_dist(ParticleFilter::weights.begin(), ParticleFilter::weights.end());
	vector<Particle> resampled_particles;
	for (int i = 0; i < ParticleFilter::num_particles; i++)
	{
		resampled_particles.push_back(particles[d_dist(DRE)]);
	}
	particles = resampled_particles;
	d_dist.reset();
}

//Just passing the best particle here.
Particle ParticleFilter::SetAssociations(Particle& best)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	vector<int> temp_associations;
	vector<double> temp_sense_x;
	vector<double> temp_sense_y;
	for (int j = 0; j < best.data_associations.size(); j++)
	{
		temp_associations.push_back(best.data_associations[j].id);
		temp_sense_x.push_back(best.data_associations[j].x);
		temp_sense_y.push_back(best.data_associations[j].y);
	}
	best.associations = temp_associations;
	best.sense_x = temp_sense_x;
	best.sense_y = temp_sense_y;

	return best;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}