# CMake generated Testfile for 
# Source directory: /home/michaaga/openvslam/test
# Build directory: /home/michaaga/openvslam/build/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_openvslam_data_common_get_cell_indices "/home/michaaga/openvslam/build/test/test_openvslam_data_common_get_cell_indices")
add_test(test_openvslam_feature_orb_extractor "/home/michaaga/openvslam/build/test/test_openvslam_feature_orb_extractor")
add_test(test_openvslam_feature_orb_params "/home/michaaga/openvslam/build/test/test_openvslam_feature_orb_params")
add_test(test_openvslam_match_angle_checker "/home/michaaga/openvslam/build/test/test_openvslam_match_angle_checker")
add_test(test_openvslam_match_base "/home/michaaga/openvslam/build/test/test_openvslam_match_base")
add_test(test_openvslam_solve_essential_solver "/home/michaaga/openvslam/build/test/test_openvslam_solve_essential_solver")
add_test(test_openvslam_solve_fundamental_solver "/home/michaaga/openvslam/build/test/test_openvslam_solve_fundamental_solver")
add_test(test_openvslam_solve_homography_solver "/home/michaaga/openvslam/build/test/test_openvslam_solve_homography_solver")
add_test(test_openvslam_solve_pnp_solver "/home/michaaga/openvslam/build/test/test_openvslam_solve_pnp_solver")
add_test(test_openvslam_util_fancy_index "/home/michaaga/openvslam/build/test/test_openvslam_util_fancy_index")
add_test(test_openvslam_util_random_array "/home/michaaga/openvslam/build/test/test_openvslam_util_random_array")
add_test(test_openvslam_util_trigonometric "/home/michaaga/openvslam/build/test/test_openvslam_util_trigonometric")
subdirs(../googletest-build)
subdirs(helper)
