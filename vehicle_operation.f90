module vehicle_operations
    use iso_c_binding
    implicit none
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

    subroutine vehicle_advence_rapper( f_fforce )
        implicit none
        interface
            subroutine vehicle_advance(c_fforce) bind(c)
                import
                real(c_double),intent(in) :: c_fforce(6)
            end subroutine vehicle_advance
        end interface
        integer :: i,j
        real(8),intent(in) :: f_fforce(6)
        real(c_double) :: c_fforce(6)

        do i =1, 6
            c_fforce(i) = f_fforce(i)
        end do
        
        !====================================================
        !if fforce matrix is two dimensions 
        !please create matrix converter Fortran to C
        !====================================================

        call vehicle_advance(c_fforce)



        return
    end subroutine vehicle_advence_rapper

end module vehicle_operations