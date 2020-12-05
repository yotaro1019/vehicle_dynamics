module vehicle_operations
    use iso_c_binding
    implicit none

    type,bind(c) :: Components
        real(c_double) :: translation(3)  !ゼロ初期化して
        real(c_double) :: rotation(3)  !ゼロ初期化して   
    end type Components

    type,bind(c) :: Cfd2Vehicle
        type(Components) fforce 
        
    end type Cfd2Vehicle    

    type,bind(c) :: Vehicle2Cfd
        type(Components) :: mesh_vel
        type(Components) :: mesh_acc
        type(Components) :: object_vel(30)
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