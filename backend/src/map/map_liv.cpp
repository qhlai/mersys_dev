// #pragma once

#include "backend/map/map_liv.hpp"



MapLIV::MapLIV(size_t id)
    : MapBase(id)
{
    associated_clients_.insert(id);
}
