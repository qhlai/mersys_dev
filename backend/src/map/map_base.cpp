#include "backend/map/map_base.hpp"


MapBase::MapBase(size_t id)
    : id_map_(id)
{
    if(id_map_ > 1000) {
        // std::cout << COUTFATAL << "Map initialized with ID " << id_map_ << std::endl;
        exit(-1);
    }
}