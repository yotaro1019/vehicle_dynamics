module vehicle_operations
    use iso_c_binding
    implicit none

    type,bind(c) :: Cfd2Vehicle
        real(c_double) :: chassis_fforce(3)  !ゼロ初期化して
        real(c_double) :: chassis_fmoment(3)  !ゼロ初期化して
        
    end type Cfd2Vehicle    

    type,bind(c) :: Vehicle2Cfd
        real(c_double) :: mesh_vel(3)
        real(c_double) :: mesh_acc(3)
        real(c_double) :: chassis_linvel(3)
        real(c_double) :: chassis_rotvel(3)
        real(c_double) :: wheel_angvel(3,20)  !Note: Rows and columns of 2D arrays are different in C and Fortran
    end type Vehicle2Cfd

contains


    subroutine vehicle_initialize_rapper()
        implicit none
        interface
            subroutine vehicle_initialize() bind(c)
            end subroutine vehicle_initialize
        end interface

        call vehicle_initialize()

        return
    end subroutine vehicle_initialize_rapper 

    subroutine vehicle_advence_rapper( cfd2veh, veh2cfd )
        implicit none
        interface
            subroutine vehicle_advance(cfd2veh, veh2cfd) bind(c)
                import
                type(Cfd2Vehicle),intent(in) :: cfd2veh
                type(vehicle2cfd),intent(out) :: veh2cfd
            end subroutine vehicle_advance
        end interface
        integer :: i,j
        type(Cfd2Vehicle),intent(in) :: cfd2veh
        type(vehicle2cfd),intent(out) :: veh2cfd

        call vehicle_advance(cfd2veh,veh2cfd)

        return
    end subroutine vehicle_advence_rapper

end module vehicle_operations