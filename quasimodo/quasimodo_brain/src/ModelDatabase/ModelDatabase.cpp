#include "ModelDatabase.h"

ModelDatabase::ModelDatabase(){storage = new ModelStorageFile();}
ModelDatabase::~ModelDatabase(){}

//Add pointcloud to database, return index number in database, weight is the bias of the system to perfer this object when searching
bool ModelDatabase::add(reglib::Model * model){
	storage->add(model);
	return true;
}
		
// return true if successfull
// return false if fail
bool ModelDatabase::remove(reglib::Model * model){
	storage->remove(model);
	return false;
}
		
//Find the number_of_matches closest matches in dabase to the pointcloud for index 
std::vector<reglib::Model *> ModelDatabase::search(reglib::Model * model, int number_of_matches){
	return std::vector<reglib::Model *>();
}
