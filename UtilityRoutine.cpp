#include "UtilityRoutine.h"

void split(char* message, Eigen::MatrixXd &covMatrix)
{
	char message_t[500];
	strncpy(message_t, message, strlen(message));
	char *p = NULL, *pNext = NULL;
	vector<char*> Cov_row;
	vector<char*> Cov_unit;

	p = strtok_r(message_t, ";", &pNext);
	while (p != NULL)
	{
		Cov_row.push_back(p);
		p = strtok_r(NULL, ";", &pNext);
	}

	for (int i = 0; i < Cov_row.size(); i++)
	{
		p = strtok_r(Cov_row[i], " ", &pNext);
		while (p != NULL)
		{
			Cov_unit.push_back(p);
			p = strtok_r(NULL, " ", &pNext);
		}
	}

	for (int i = 0; i < Cov_row.size(); i++)
	{
		for (int j = 0; j < Cov_row.size(); j++)
		{
			double test = atof(Cov_unit[Cov_row.size()*i + j]);
			covMatrix(i, j) = atof(Cov_unit[Cov_row.size()*i + j]);
		}
	}
}

void split_vector(char* message, Eigen::VectorXd &Vector)
{
	char message_t[500];
	strncpy(message_t, message, strlen(message));
	char* p = NULL, *pNext = NULL;
	vector<char*> elements;
	p = strtok_r(message_t, " ", &pNext);
	while (p != NULL)
	{
		elements.push_back(p);
		p = strtok_r(NULL, " ", &pNext);
	}
	for (int i = 0; i < elements.size(); i++)
	{
		Vector(i) = atof(elements[i]);
	}
}
