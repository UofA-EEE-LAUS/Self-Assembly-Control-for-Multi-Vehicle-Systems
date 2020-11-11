  Author:Yangkawaiwei Zhou
  Email:a1759122@student.adelaide.edu.au
  
  The Formation Control of this project is using visual leader-follower method to solve the problem. 
  
  The function apfpath.m gives a basic concept of how to generating a collision-free path within n direction, and
the collision-free path will be consdered as the visual leader of the desired structure.  
  
  The function DCMline.m is for generating the line structure from the path and rotates the structure to global coordination.
  The function DCMTri.m is for generating the triangle structure from the path and rotates the structure to global coordination.

  The function pairing.m is used to solve the assignment problem of bipartite graph using Hungarian Algorithm, 
the toolbox function munkres.m is used to generate the adjacency matrix of the result of the assignment.  

  The main fuction of line formation is main_line.m and the main function of triangle formation is mian_tri.m,
before using these two function, run pathresult.mat first to get the stored collision-free path for the V-REP scene
scene_three_rover_finaltest_camera.ttt, the main function is used for moving the multi-vehicles system in the V-REP
scene.
