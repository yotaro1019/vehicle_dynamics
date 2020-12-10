program example_coupling
    use vehicle_operations
    use mpi
    implicit none
    integer :: call_itvl
    integer :: begin_step
    real(8) :: cube_dt, chrono_dt
    real(8) :: t_begin, t_end
    integer :: i
    real(8) :: cube_time
    integer :: cube_step
    real(8) :: cube_fforce(6)
    real(8) :: fforce
    real(8) :: pi
    integer :: output_id
    integer :: loop_begin, loop_end, loop
    type(Cfd2Vehicle),target :: cfd2veh
    type(vehicle2cfd),target :: veh2cfd
    integer :: ierr, PETOT, my_rank
    pi = 4.0 * atan(1.0)

    !=========================================
    !parameters
    cube_dt = 0.001
    chrono_dt = 0.001
    call_itvl = 1
    t_begin = 0.0
    t_end = 8.0
    begin_step = 0
    !=========================================

    call MPI_INIT(ierr)
    call MPI_COMM_SIZE(MPI_COMM_WORLD, PETOT, ierr)
    call MPI_COMM_RANK(MPI_COMM_WORLD, my_rank, ierr)  
    call MPI_Barrier( MPI_COMM_WORLD, ierr) 
    
    if(my_rank == 0)then   
        call vehicle_initialize_rapper()    !initialize vehicle system
    end if

    cube_time = t_begin    
    cube_step  = begin_step

    loop_begin = 1 + int(t_begin/cube_dt)
    loop_end = int(t_end/cube_dt)



    write(*,*) "=========================================================================="
    do loop = 1+cube_step, loop_end 
        call MPI_Barrier( MPI_COMM_WORLD, ierr) 
        !--------------------------------------------------------
        !cube flow calculation
        cube_time = cube_time + cube_dt
        cube_step = cube_step+1


        fforce = 100*sin(pi * cube_step/15000)
        
        cfd2veh%cfd_time = cube_time
        cfd2veh%fforce%translation(:) = 10.0 !fforce
        cfd2veh%fforce%rotation(:) = 10.0 !fforce      


        !--------------------------------------------------------
        call MPI_Barrier( MPI_COMM_WORLD, ierr) 
        if(mod(cube_step,call_itvl) == 0)then
            

            
            if(my_rank == 0)then            
                call  vehicle_advence_rapper(cfd2veh, veh2cfd)
                write(*,*) "chrono_call"
                write(*,*) "my_rank = ", my_rank, " / " , PETOT, "loop = ", loop
                write(*,*) "<translation>"
                write(*,*) veh2cfd%mesh_vel%translation(1),veh2cfd%mesh_vel%translation(2),veh2cfd%mesh_vel%translation(3)
            end if
            
        end if
        call MPI_Barrier( MPI_COMM_WORLD, ierr);   
        call veh2cfd_mpi_bcast(veh2cfd)

        write(*,*) ":*my_rank = ", my_rank,"loop = ", loop,  veh2cfd%mesh_vel%translation(1),&
            veh2cfd%mesh_vel%translation(2),veh2cfd%mesh_vel%translation(3)
        call MPI_Barrier( MPI_COMM_WORLD, ierr);
        if(my_rank == 0)then            
            write(*,*) "**********************************"
        end if
        call MPI_Barrier( MPI_COMM_WORLD, ierr);

    end do
        call MPI_FINALIZE(ierr)
        return 


    stop
end program example_coupling

