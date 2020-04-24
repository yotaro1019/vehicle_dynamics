module vehicle_operations
    use iso_c_binding
    implicit none

    type,bind(c) :: vehicle2cfd
        real(c_float) :: mesh_vel(3)
        real(c_float) :: mesh_acc(3)
    end type vehicle2cfd

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
        type(vehicle2cfd) :: veh2cfd


        
        !====================================================
        !if fforce matrix is two dimensions 
        !please create matrix converter Fortran to C
        !====================================================
        write(*,*) "veh2cfd%mesh_vel--3    ", veh2cfd%mesh_vel(1), veh2cfd%mesh_vel(2), veh2cfd%mesh_vel(3)
        write(*,*) "veh2cfd%mesh_acc--3    ", veh2cfd%mesh_acc(1), veh2cfd%mesh_acc(2), veh2cfd%mesh_acc(3)
        write(*,*)
        call vehicle_advance(fforce,veh2cfd)
        write(*,*) "veh2cfd%mesh_vel--4    ", veh2cfd%mesh_vel(1), veh2cfd%mesh_vel(2), veh2cfd%mesh_vel(3)
        write(*,*) "veh2cfd%mesh_acc--4    ", veh2cfd%mesh_acc(1), veh2cfd%mesh_acc(2), veh2cfd%mesh_acc(3)
        write(*,*)


        return
    end subroutine vehicle_advence_rapper

end module vehicle_operations