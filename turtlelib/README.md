# Turtlelib Library
A library for handling transformations in SE(2), forward and inverse kinematics for a differential drive robot. The library includes 
a Transform2D class and a DiffDrive class which help compact the necessary mathematical equations. It also contains an EKF (Extended Kalman Filter) class that handles all the necessary calculations for implementing the filter for SLAM.

# Components
- rigid2d - Handles 2D rigid body transformations
- diff_drive - Handles the kinematics for a differential drive robot. The mathematical derivations can be found in the `docs` folder of the package.
- frame_main - Perform some rigid body computations based on user input

Example output of frame_main:

        Enter transform T_{a,b}: 
        50 1 2
        Enter transform T_{b,c}: 
        70 0 3
        T_{a,b}: deg: 50 x: 1 y: 2
        T_{b,a}: deg: -50 x: -2.17488 y: -0.519531
        T_{b,c}: deg: 70 x: 0 y: 3
        T_{c,b}: deg: -70 x: -2.81908 y: -1.02606
        T_{a,c}: deg: 120 x: -1.29813 y: 3.92836
        T_{c,a}: deg: -120 x: -4.05113 y: 0.839965
        Enter vector v_b: 
        2 4
        v_bhat: [0.447214 0.894427]
        v_a: [-0.778603 6.10324]
        v_b: [2 4]
        v_c: [1.62373 -1.53737]
        Enter twist V_b: 
        3 2 1
        v_a: [3 6.51953 -0.825124]
        V_b: [3 2 1]
        V_c: [3 -1.45445 6.91987]

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
        - You could implement it in the Transform2D class, in the Vector2D struct, or as a helper function in the namespace

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
        - According to the guidelines you should only include a function in as a member if it needs direct access to the representation of the class. The normalize function does not need to access the class's respresentation so it should not be a member. The function could be added to the struct, but adding it outside as a helper function makes it easier to understand that it also supports the class. 
    
   - Which of the methods would you implement and why?
        - I would integrate the normalize function as a helper function in the namespace since it is just using the data in the struct, but it can still be useful to the overall transform2D class. 

2. What is the difference between a class and a struct in C++?

    - By default a struct has its member variables as public while a class has them as private unless assigned otherwise. 


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
    - Vector2D is a struct because it is only used to store the data that makes up a 2D vector (C.1), whereas Transform2D includes much more functionality that uses its members. It also makes more sense for Transform2D to be a class since you want its members to be private. This ensures that you do not accidentally access the vector in the transform class rather than a new vector you are trying to create or change (C.4).


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
    - Single argument constructors should be set to explicit by default. This avoids any implicit type conversions (C.46).


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

      - The inv() function is delcared const because you do not want the transform2D objects values to be changed after calling the function. However, for the *= operator you want to store the result in the object, meaning you want to change the values of the object. To do so they must not be set as const so that they are mutable. 
