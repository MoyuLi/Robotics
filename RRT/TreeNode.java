//package com.turingvideo.robot.algo;

import java.util.ArrayList;

public class TreeNode {

    private double x;
    private double y;
    private double cost;
    private TreeNode parent;
    private ArrayList<TreeNode> children;
    private ArrayList<TreeNode> parents;

    public TreeNode () {
        x = 0;
        y = 0;
        cost = 0;
        parent = null;
        children = new ArrayList <TreeNode> ();
        parents = new ArrayList <TreeNode> ();
    }

    public TreeNode (double x, double y) {
        this.x = x;
        this.y = y;
        cost = 0;
        parent = null;
        children = new ArrayList <TreeNode> ();
        parents = new ArrayList <TreeNode> ();
    }

    public ArrayList <TreeNode> getChildren () {
        return children;
    }

    public TreeNode getParent() {
        return parent;
    }

    public ArrayList<TreeNode> getParents() {
        return parents;
    }

    public double getCost() {
        return cost;
    }

    public void setParent (TreeNode parent) {
        this.parent = parent;
    }

    public void addChild (TreeNode child) {
        this.children.add(child);
    }

    public void addParent (TreeNode parent) {
        this.parents.add(parent);
    }

    public void setCost (double cost) {
        this.cost = cost;
    }

    public void set (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX () {
        return this.x;
    }

    public double getY () {
        return this.y;
    }

}





