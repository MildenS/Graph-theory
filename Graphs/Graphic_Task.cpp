#include "Graphic_Task.h"
#include<memory>
#include <SFML/Graphics.hpp>
#include<unordered_set>
#include<set>

Graphic_Task::Graphic_Task(std::string graph_path)
{
    graph = std::make_unique<Graph>(graph_path);
    MST = graph->MST();
    current_graph = TypeOfGraph::Graph;
}

void Graphic_Task::Work()
{
    using Edge = std::pair<std::string, std::string>;
    const int height = 1080, width = 1920, node_radius = 20, nodes_count = graph->GetVertexCount();
    const int max_lines_count = nodes_count * (nodes_count - 1) / 2;
    const float mid = static_cast<float>(width) / 2.;
	sf::RenderWindow window(sf::VideoMode(width, height), "Graph");
    sf::VertexArray lines(sf::Quads /*,количество точек*/);

    //get ready to draw graph
    std::string root = graph->GetFirstVertex();
    auto cur_adj = graph->GetAdjacentyListOfVertex(root);
    auto vertecies = graph->GetAllVertecies();
    std::unordered_map<std::string, bool> isChecked;
    //name - Node
    std::unordered_map<std::string, Node> nodes;
    for (const auto& v : vertecies)
    {
        isChecked.insert(std::make_pair(v, false));
        auto cur_adj = graph->GetAdjacentyListOfVertex(v);
        nodes.insert(std::make_pair(v, Node{ 0., 0., v, cur_adj, false }));
    }
    std::set<Edge> drawn_edges;
        
    isChecked[root] = true;
    Node cur_node{ mid, 20, root, cur_adj, true, 20};
    nodes[root] = cur_node;
    std::queue<Node> q;
    const float level_height = 100;
    q.push(cur_node);
    while (!q.empty())
    {
        cur_node = q.front();
        q.pop();
        if (isChecked[cur_node.name] && cur_node.name != root)
            continue;
        isChecked[cur_node.name] = true;
        uint16_t sons_count = cur_node.adj.size();
        const float max_width = 1000.f;
        const float radius_of_son = 40;
        const float space = (max_width - radius_of_son * sons_count) / (sons_count - 1);
        uint16_t son_number = 0;
        Node prev_son;
        for (const auto& son : cur_node.adj)
        {
            auto& son_node = nodes[son.first];
            if (nodes[son.first].is_ready &&
                drawn_edges.find(std::make_pair(cur_node.name, son_node.name))
                == drawn_edges.end() &&
                drawn_edges.find(std::make_pair(son_node.name, cur_node.name))
                == drawn_edges.end())
            {
                //continue;
                lines.append(sf::Vertex(sf::Vector2f(cur_node.x - 2.5 + cur_node.radius,
                    cur_node.y + cur_node.radius), sf::Color::Blue));
                lines.append(sf::Vertex(sf::Vector2f(son_node.x - 2.5 + cur_node.radius,
                    son_node.y + cur_node.radius), sf::Color::Blue));
                lines.append(sf::Vertex(sf::Vector2f(son_node.x + 2.5 + cur_node.radius,
                    son_node.y + cur_node.radius + 2.5), sf::Color::Blue));
                lines.append(sf::Vertex(sf::Vector2f(cur_node.x + 2.5 + cur_node.radius,
                    cur_node.y + cur_node.radius + 2.5), sf::Color::Blue));
                drawn_edges.insert(std::make_pair(cur_node.name,
                    son_node.name));
                continue;
            }
            else if (nodes[son.first].is_ready)
                continue;
            if (son_number == 0)
            {
                son_node.x = cur_node.x - max_width / 2 + radius_of_son / 2;
                son_node.y = cur_node.y + level_height;
            }
            else
            {
                son_node.x = prev_son.x + space;
                son_node.y = prev_son.y;
            }
            son_node.radius = radius_of_son;
            son_node.is_ready = true;
            ++son_number;
            prev_son = son_node;
            lines.append(sf::Vertex(sf::Vector2f(cur_node.x - 2.5 + cur_node.radius,
                cur_node.y + cur_node.radius - 2.5), sf::Color::Blue));
            lines.append(sf::Vertex(sf::Vector2f(son_node.x - 2.5 + cur_node.radius,
                son_node.y + cur_node.radius - 2.5), sf::Color::Blue));
            lines.append(sf::Vertex(sf::Vector2f(son_node.x + 2.5 + cur_node.radius,
                son_node.y + cur_node.radius + 2.5), sf::Color::Blue));
            lines.append(sf::Vertex(sf::Vector2f(cur_node.x + 2.5 + cur_node.radius,
                cur_node.y + cur_node.radius + 2.5), sf::Color::Blue));
            drawn_edges.insert(std::make_pair(cur_node.name,
                son_node.name));
            q.push(son_node);

        }
        //nodes[cur_node.name].is_ready = true;
    }




    sf::VertexArray mst_lines(sf::Quads /*,количество точек*/);

    auto mst_edges = graph->MST_edges();
    


    sf::Font MyFont;
    MyFont.loadFromFile("HelgacBolditalic.otf");
    sf::CircleShape shape(40.f);
    shape.setFillColor(sf::Color::Green);
    shape.setPosition(sf::Vector2f(mid - 40 / 2, 20));
    uint16_t mst_edges_to_draw = 0;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyReleased)
                if (event.key.code == sf::Keyboard::Space)
                {
                    if (current_graph == TypeOfGraph::Graph)
                    {
                        current_graph = TypeOfGraph::MST;
                        mst_edges_to_draw = 0;
                    }
                    else
                    {
                        current_graph = TypeOfGraph::Graph;
                        mst_edges_to_draw = 0;
                    }
                }
                else if (event.key.code == sf::Keyboard::Right && current_graph == TypeOfGraph::MST
                    && mst_edges_to_draw < mst_edges.size())
                    ++mst_edges_to_draw;
        }
        

        window.clear(sf::Color::White);


        
        if (current_graph == TypeOfGraph::Graph)
        {
            window.draw(lines);
            window.draw(shape);
            sf::Text name;
            name.setFont(MyFont);
            name.setFillColor(sf::Color::Black);
            name.setStyle(sf::Text::Bold);
            for (const auto& node : nodes)
            {
                sf::CircleShape shape(node.second.radius);
                shape.setFillColor(sf::Color::Green);
                shape.setPosition(sf::Vector2f(node.second.x, node.second.y));
                window.draw(shape);
                name.setString(node.second.name);
                name.setPosition(node.second.x + node.second.radius / 2,
                    node.second.y + node.second.radius / 2);
                window.draw(name);
            }
            for (const auto& edge : drawn_edges)
            {
                auto first_node = nodes[edge.first];
                auto second_node = nodes[edge.second];
                float text_height = name.getLocalBounds().height;
                float text_width = name.getLocalBounds().width;
                float x_position = std::abs((first_node.x - second_node.x)) / 2 +
                    +std::min(first_node.x, second_node.x) +text_width / 2;
                name.setString(std::to_string(first_node.adj[second_node.name]));
                
                float y_position = std::abs((first_node.y - second_node.y)) / 2 +
                    std::min(first_node.y, second_node.y) + text_height / 2;
                //name.setString(std::to_string(first_node.adj[second_node.name]));
                name.setPosition(x_position, y_position);
                window.draw(name);
            }
            sf::Text subscript;
            subscript.setFont(MyFont);
            subscript.setString("Graph");
            subscript.setFillColor(sf::Color::Black);
            subscript.setStyle(sf::Text::Bold);
            subscript.setPosition(width - 100,
                height - 50);
            window.draw(subscript);
        }
        else if (current_graph == TypeOfGraph::MST)
        {
            sf::VertexArray mst_lines(sf::Quads /*,количество точек*/);

            for (size_t i = 0; i < mst_edges_to_draw && i < mst_edges.size(); ++i)
            {
                auto first_node = nodes[mst_edges[i].second.first];
                auto second_node = nodes[mst_edges[i].second.second];

                mst_lines.append(sf::Vertex(sf::Vector2f(first_node.x - 2.5 + first_node.radius,
                    first_node.y + first_node.radius - 2.5), sf::Color::Blue));
                mst_lines.append(sf::Vertex(sf::Vector2f(second_node.x - 2.5 + first_node.radius,
                    second_node.y + first_node.radius - 2.5), sf::Color::Blue));
                mst_lines.append(sf::Vertex(sf::Vector2f(second_node.x + 2.5 + first_node.radius,
                    second_node.y + first_node.radius + 2.5), sf::Color::Blue));
                mst_lines.append(sf::Vertex(sf::Vector2f(first_node.x + 2.5 + first_node.radius,
                    first_node.y + first_node.radius + 2.5), sf::Color::Blue));
            }

            window.draw(mst_lines);
            window.draw(shape);
            sf::Text name;
            name.setFont(MyFont);
            name.setFillColor(sf::Color::Black);
            name.setStyle(sf::Text::Bold);
            
            for (const auto& node : nodes)
            {
                sf::CircleShape shape(node.second.radius);
                shape.setFillColor(sf::Color::Green);
                shape.setPosition(sf::Vector2f(node.second.x, node.second.y));
                window.draw(shape);
                name.setString(node.second.name);
                name.setPosition(node.second.x + node.second.radius / 2,
                    node.second.y + node.second.radius / 2);
                window.draw(name);
            }


            for (size_t i = 0; i < mst_edges_to_draw && i < mst_edges.size(); ++i)
            {
                auto first_node = nodes[mst_edges[i].second.first];
                auto second_node = nodes[mst_edges[i].second.second];

                float text_height = name.getLocalBounds().height;
                float text_width = name.getLocalBounds().width;
                float x_position = std::abs((first_node.x - second_node.x)) / 2 +
                    +std::min(first_node.x, second_node.x) + text_width / 2;
                name.setString(std::to_string(first_node.adj[second_node.name]));

                float y_position = std::abs((first_node.y - second_node.y)) / 2 +
                    std::min(first_node.y, second_node.y) + text_height / 2;
                //name.setString(std::to_string(first_node.adj[second_node.name]));
                name.setPosition(x_position, y_position);
                window.draw(name);
            }
            

            window.draw(name);
            sf::Text subscript;
            subscript.setFont(MyFont);
            uint16_t remain_edges = mst_edges.size() - mst_edges_to_draw;
            std::string remain_string = std::string("Remain ") + std::to_string(remain_edges) +
                std::string(" MST edges");
            subscript.setString(remain_string);
            subscript.setFillColor(sf::Color::Black);
            subscript.setStyle(sf::Text::Bold);
            subscript.setPosition(width - 320,
                height - 50);
            window.draw(subscript);
        }
        
        window.display();
    }

}

