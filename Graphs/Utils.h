#pragma once
#include<vector>
#include<string>
#include"boost/tokenizer.hpp"
#include"CoreTypes.h"

//todo: use boost

inline std::vector<std::string> split(std::string in, std::string delims)
{
    std::vector<std::string > data;

    boost::char_separator<char> sep(delims.c_str());
    boost::tokenizer < boost::char_separator<char>> tokenizer(in, sep);

    for (auto tok_iter = tokenizer.begin(); tok_iter != tokenizer.end(); ++tok_iter)
        data.push_back(*tok_iter);

    return data;
}

inline std::string fromTypeToString(GraphType type)
{
    if (type == GraphType::Empty)
        return "Empty";
    else if (type == GraphType::UnweightedOriented)
        return "UnweightedOriented";
    else if (type == GraphType::UnweightedUndirected)
        return "UnweightedUndirected";
    else if (type == GraphType::WeightedOriented)
        return "WeightedOriented";
    else if (type == GraphType::WeightedUndirected)
        return "WeightedUndirected";
    else
        throw GraphException("wrong graph type");
}

inline void deleteByValue(std::vector<Graph>& v, const std::string& value)
{
    for (size_t i = 0; i < v.size(); i++)
    {
        if (v[i].GetName() == value)
        {
            v.erase(v.begin() + i);
            break;
        }
    }
}

