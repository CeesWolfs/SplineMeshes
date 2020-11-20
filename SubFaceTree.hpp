#pragma once
#include "Types.hpp"
#include <vector>
#include <numeric>
#include <stdint.h>
#include <cstdio>
#include <string>
#include <fstream>

typedef struct _node
{
    halfFace parent;
    float split_coord;
    enum axis split_axis;
    halfFace lower_child;
    halfFace top_child;
} Node;

class SubFaceTree
{
private:
    
public:
    std::vector<Node> nodes; 
    SubFaceTree(/* args */);
    static uint32_t toNodeIndex(halfFace from) {return from.id >> 3;}
    ~SubFaceTree();
};

static inline bool isHigher(halfFace face) {
    face.getLocalId() == 7;
}

class SubFaceIterator
    {
    private:
        const SubFaceTree & tree;
        uint32_t node_index;
        bool at_lower;
    public:
        const halfFace& operator*() {
            return at_lower ? tree.nodes[node_index].lower_child : tree.nodes[node_index].top_child;
        }
        SubFaceIterator& begin(uint32_t start_node_index) {
            node_index = start_node_index;
            at_lower = true;
            while(tree.nodes[node_index].lower_child.isSubdivided()) {
                node_index = tree.toNodeIndex(tree.nodes[node_index].lower_child);
            }
            return *this;
        }
        SubFaceIterator end() {
            return {tree, -1, false};
        }
        const SubFaceIterator& operator++() {
            if (at_lower) {
                at_lower = false;
                if(tree.nodes[node_index].top_child.isSubdivided()) {
                    node_index = tree.toNodeIndex(tree.nodes[node_index].top_child);
                    // Move down until lower child no longer split
                    while(tree.nodes[node_index].lower_child.isSubdivided()) {
                        node_index = tree.toNodeIndex(tree.nodes[node_index].lower_child);
                    }
                }
                return *this;
            }
            // We are already at the higher element, move up the chain
            at_lower = !isHigher(tree.nodes[node_index].parent);
            while(!at_lower) {
                node_index = tree.toNodeIndex(tree.nodes[node_index].parent);
            }

        }
    }


