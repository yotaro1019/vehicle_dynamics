module vehicle_operations
    use iso_c_binding
    implicit none
    type,bind(c) :: Vehicle2Cfd
        real(c_double) :: mesh_vel(3)
        real(c_double) :: mesh_acc(3)
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

    subroutine vehicle_advence_rapper( fforce, veh2cfd )
        implicit none
        interface
            subroutine vehicle_advance(fforce, veh2cfd) bind(c)
                import
                real(c_double),intent(in) :: fforce(6)
                type(vehicle2cfd),intent(out) :: veh2cfd
            end subroutine vehicle_advance
        end interface
        integer :: i,j
        real(c_double),intent(in) :: fforce(6)
        type(vehicle2cfd),intent(out) :: veh2cfd

        call vehicle_advance(fforce,veh2cfd)

        return
    end subroutine vehicle_advence_rapper

end module vehicle_operations