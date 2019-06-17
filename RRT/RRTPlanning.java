// package com.turingvideo.robot.algo;
//
// import android.graphics.Bitmap;
// import android.graphics.BitmapFactory;
// import android.os.Environment;
// import android.util.Log;

// import java.io.File;
// import java.io.FileOutputStream;
// import java.io.IOException;
import java.util.ArrayList;
//import TreeNode;


public class RRTPlanning {

    private double stepThreshold = 1.5;
    private int numOfVertices = 30;
    private double nearThreshold = 3;

    //private Bitmap map;
    private static double [][] map;
    private TreeNode start, goal;
    private static ArrayList<TreeNode> list;


    public RRTPlanning(TreeNode start, TreeNode goal, double[][] map) {
        this.map = map;
        this.start = start;
        this.goal = goal;
        list = new ArrayList<>();
        list.add(start);

    }

    public static void main (String args []) {

        // String baseDir = Environment.getExternalStorageDirectory().toString() + "/android_robot";
        // String filename = baseDir + "/rrtmap/map1.png";
        // Bitmap map = BitmapFactory.decodeFile(filename);
        //
        // Point2D start = new Point2D(map.getWidth()/2,map.getHeight()/2);
        // Point2D goal = new Point2D(map.getWidth()*2/3,map.getHeight()/2);

        map = new double [][] {
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 1, 1, 1, 1, 1, 1, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
        };

        TreeNode startNode = new TreeNode(0,0);
        startNode.setCost(0);
        TreeNode goalNode = new TreeNode(4, 9);

        RRTPlanning rrt = new RRTPlanning(startNode, goalNode, map);
        rrt.Build_RRT_STAR();
        ArrayList <TreeNode> path = rrt.getPath();

        for (TreeNode node : path) {
          System.out.println(node.getX()+", "+node.getY());
        }

        // for (TreeNode node : list) {
        //   System.out.print(node.getX()+", "+node.getY());
        // }

        //return rrt.getPath();

        //ArrayList<Point2D> path = rrt(map, start, goal);

        // for (Point2D point : path) {
        //     System.out.print(point);
        // }

        //drawPath(map,null);

    }

    // public static ArrayList<Point2D> rrt(Bitmap map, Point2D start, Point2D goal) {
    //     TreeNode startNode = new TreeNode(start.x,start.y);
    //     startNode.setCost(0);
    //     TreeNode goalNode = new TreeNode(goal.x, goal.y);
    //
    //     RRTPlanning rrt = new RRTPlanning(startNode, goalNode, map);
    //     rrt.Build_RRT();
    //
    //     return rrt.getPath();
    // }

    public ArrayList<TreeNode> getPath () {

        double minDistance = 100000;

        for (TreeNode node : list) {

            double goalDistance = calcDistance(node.getX(),goal.getX(),node.getY(),goal.getY());

            if (goalDistance < minDistance) {
                minDistance = goalDistance;
                goal.setParent(node);
            }
        }

        //ArrayList<Point2D> path = new ArrayList<>();
        //path.add(new Point2D(goal.getX(),goal.getY()));

        ArrayList<TreeNode> path = new ArrayList<>();
        path.add(goal);

        TreeNode curr = goal;

        while (curr.getParent()!=null) {
            curr = curr.getParent();
            path.add(curr);
            //path.add(new Point2D(curr.getX(),curr.getY()));
        }

        return path;
    }

    public double calcDistance (double x1, double x2, double y1, double y2) {
      return Math.sqrt( Math.pow((x1-x2),2) + Math.pow((y1-y2),2) );
    }

    public void Build_RRT() {

        for (int i = 0; i<numOfVertices; i++) {

            System.out.println("num of vertices: " + i);

            TreeNode randNode = rand_conf();

            TreeNode nearestNode = nearest(randNode, list);

            TreeNode newNode = steer(randNode, nearestNode);

            if (collisionFree(newNode, nearestNode)) {

                list.add(newNode);

                newNode.setParent(nearestNode);

                double distance = calcDistance(nearestNode.getX(),newNode.getX(),nearestNode.getY(),newNode.getY());

                newNode.setCost(distance + nearestNode.getCost());

            }

            if (calcDistance(newNode.getX(),goal.getX(),newNode.getY(),goal.getY()) < 1.5) {
                break;
            }

        }
    }

    private TreeNode nearest (TreeNode randNode, ArrayList<TreeNode> list ) {

        double minDistance = 100000;
        TreeNode nearestNode = null;

        for (TreeNode tempNode : list) {

            double tempDistance = calcDistance(tempNode.getX(),
                    randNode.getX(),tempNode.getY(),randNode.getY());

            if (tempDistance < minDistance) {
                minDistance = tempDistance;
                nearestNode = tempNode;
            }
        }
        return nearestNode;
    }

    private TreeNode steer (TreeNode randNode, TreeNode nearestNode) {

        double distance = calcDistance(nearestNode.getX(),
                randNode.getX(),nearestNode.getY(),randNode.getY());

        if (distance > stepThreshold) {

            double x = nearestNode.getX() +
                    (randNode.getX()-nearestNode.getX()) * stepThreshold / distance;

            double y = nearestNode.getY() +
                    (randNode.getY()-nearestNode.getY()) * stepThreshold / distance;

            double theta = 0;

            TreeNode steerNode = new TreeNode(x,y);

            return steerNode;

        }
        else {
            return randNode;
        }
    }

    private TreeNode rand_conf () {

        TreeNode randNode = new TreeNode( Math.random()*4, Math.random()*9 );

        while (!isValid(randNode)) {

            //double x = Math.random()*map.getWidth();
            //double y = Math.random()*map.getHeight();

            double x = Math.random()*4;
            double y = Math.random()*9;

            randNode.set(x,y);

        }

        return randNode;
    }

    public boolean isValid (TreeNode node) {
        int x = (int) (node.getX());
        int y = (int) (node.getY());

        if (map[x][y] != 1) {
            return true;
        }
        return false;
    }

    private boolean collisionFree(TreeNode newNode, TreeNode nearestNode) {

        ArrayList<Point2D> collisionPath = new ArrayList<>();

        collisionPath.add(new Point2D(newNode.getX(),newNode.getY()) );
        collisionPath.add(new Point2D(nearestNode.getX(),nearestNode.getY()) );

        ArrayList<Point2D> interpolatedCollisionPath = interpolate(collisionPath, 0.5);

        for (Point2D point : interpolatedCollisionPath) {
            if (map[(int)point.x][(int)point.y] == 1 ) {
                return false;
            }
        }
        return true;

    }

    private ArrayList<TreeNode> near (TreeNode newNode) {

        ArrayList<TreeNode> nearNodes = new ArrayList<>();

        for (TreeNode node : list) {

            if (!collisionFree(newNode, node) &&
                    calcDistance(newNode.getX(),node.getX(),newNode.getY(),node.getY()) <= nearThreshold) {

                nearNodes.add(node);

            }
        }

        return nearNodes;
    }

    public ArrayList<Point2D> interpolate(ArrayList<Point2D> path, double stepSize) {
        ArrayList <Point2D> interpolatedPath = new ArrayList<>();
        int n = path.size();
        int m;

        if (stepSize == -1) {
            stepSize = 0.1;
        }

        //interpolatedPath.add(new Point2D(path.get(0).x, path.get(0).y));

        for (int i = 0; i<n-1; i++) {
            double currx = path.get(i).x;
            double curry = path.get(i).y;
            double nextx = path.get(i+1).x;
            double nexty = path.get(i+1).y;

            m = (int) (calcDistance(currx,nextx,curry,nexty)/stepSize);

            for (int j = 0; j<m; j++) {
                double addx = (double)(m-j)/m*currx + (double)j/m*nextx;
                double addy = (double)(m-j)/m*curry + (double)j/m*nexty;
                interpolatedPath.add(new Point2D(addx,addy));
            }
        }

        interpolatedPath.add(new Point2D(path.get(n-1).x, path.get(n-1).y));

        return interpolatedPath;
    }

    public void Build_RRG() {

        for (int i = 0; i<numOfVertices; i++) {

            System.out.println("num of vertices: " + i);

            TreeNode randNode = rand_conf();

            TreeNode nearestNode = nearest(randNode, list);

            TreeNode newNode = steer(randNode, nearestNode);

            System.out.println(newNode.getX()+", "+newNode.getY());

            if (collisionFree(newNode, nearestNode)) {

                list.add(newNode);

                newNode.addParent(nearestNode);

                nearestNode.addChild(newNode);

                ArrayList<TreeNode> nearNodes = near(newNode);

                newNode.setParent(nearestNode);

                for (TreeNode nearNode : nearNodes) {

                    if (collisionFree(newNode, nearNode) ) {

                        newNode.addParent(nearNode);

                        nearNode.addChild(newNode);
                    }
                }
            }

            if (calcDistance(newNode.getX(),goal.getX(),newNode.getY(),goal.getY()) < 0.1) {
                break;
            }

        }
    }

    public void Build_RRT_STAR() {

        for (int i = 0; i<numOfVertices; i++) {

            System.out.println("num of vertices: " + numOfVertices);

            TreeNode randNode = rand_conf();

            TreeNode nearestNode = nearest(randNode, list);

            TreeNode newNode = steer(randNode, nearestNode);

            if (collisionFree(newNode, nearestNode)) {

                list.add(newNode);

                ArrayList<TreeNode> nearNodes = near(newNode);

                double bestCost = calcDistance(nearestNode.getX(),newNode.getX(),nearestNode.getY(),newNode.getY()) + nearestNode.getCost();

                TreeNode bestNeighbor = nearestNode;

                for (TreeNode nearNode : nearNodes) {

                    if (collisionFree(newNode, nearNode) ) {

                        if (nearNode.getCost() +
                                calcDistance(nearNode.getX(),newNode.getX(),nearNode.getY(),newNode.getY()) < bestCost) {

                            bestCost = nearNode.getCost() + calcDistance(nearNode.getX(),newNode.getX(),nearNode.getY(),newNode.getY());

                            bestNeighbor = nearNode;

                        }
                    }
                }

                newNode.setParent(bestNeighbor);

                newNode.setCost(bestCost);

                // rewire
                for (TreeNode nearNode : nearNodes) {

                    if (nearNode == newNode.getParent()) {
                        continue;
                    }

                    if (collisionFree(newNode, nearNode) &&
                            nearNode.getCost() > newNode.getCost() +
                                    calcDistance(newNode.getX(), nearNode.getX(),newNode.getY(),nearNode.getY())) {

                        nearNode.setParent(newNode);
                    }
                }

            }

            if (calcDistance(newNode.getX(),goal.getX(),newNode.getY(),goal.getY()) < 0.1) {
                break;
            }

        }
    }

    // public static void drawPath(final Bitmap map, Path path) {
    //     new Thread(new Runnable() {
    //         @Override
    //         public void run() {
    //             String baseDir = Environment.getExternalStorageDirectory().toString() + "/android_robot";
    //             String rrtDir = baseDir + "/rrtmap";
    //             new File(rrtDir).mkdirs();
    //
    //             String pathName = rrtDir + "/mapResult1.png";
    //             File f = new File(pathName);
    //             try {
    //                 Log.e("drawing","");
    //                 FileOutputStream fOut = new FileOutputStream(f);
    //                 map.compress(Bitmap.CompressFormat.PNG, 100, fOut);
    //                 fOut.flush();
    //                 fOut.close();
    //             } catch (IOException e) {
    //                 if (DEBUG) {
    //                     Log.e("RRT", "Can not open file for occupancy map visualization", e);
    //                 }
    //             }
    //         }
    //     }).start();
    // }

}
