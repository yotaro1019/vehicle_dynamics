program example_coupling
    use vehicle_operations
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
    type(vehicle2cfd),target :: veh2cfd

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


    call vehicle_initialize_rapper()    !initialize vehicle system
    
    cube_time = t_begin    
    cube_step  = begin_step
    call file_open()
    if(cube_time == 0.0)then
        cube_fforce(:) = 0.0
    end if

    if(cube_time < t_end)then
        call  vehicle_advence_rapper(cube_fforce, veh2cfd)
    end if

    loop_begin = 1 + int(t_begin/cube_dt)
    loop_end = int(t_end/cube_dt)
    do loop = 1+cube_step, loop_end 
        !--------------------------------------------------------
        !cube flow calculation
        cube_time = cube_time + cube_dt
        cube_step = cube_step+1

        !if(cube_time > t_end)then
        !    exit
        !end if

        fforce = 100*sin(pi * cube_step/15000)
        do i = 1,6
            cube_fforce(i) = 0.0d0 !i * fforce
        end do

        

        !write(*,'(a10,i10,a15,f10.5)') "cube_step", cube_step, "cube_time", cube_time
        !--------------------------------------------------------
        if(mod(cube_step,call_itvl) == 0)then
            write(10,'(i10,7f15.5)') cube_step, cube_time, cube_fforce(1), cube_fforce(2), cube_fforce(3), &
            &cube_fforce(4), cube_fforce(5), cube_fforce(6)
            
            
            call  vehicle_advence_rapper(cube_fforce, veh2cfd)

            !call file_write(cube_time,  mesh_vel_acc, chassis_vel_cube, str_vel_cube, wheel_rot_cube )

        end if   



    end do

    call file_close()



    stop
end program example_coupling



subroutine file_open()
    implicit none
    character fname*100
    integer :: i
    open(10, file = "fforce.inp", status = "unknown")
    write(10,'(a10,7a15)') 'step','time', 'fx', 'fy', 'fz', 'mx', 'my', 'mz'

    call system("mkdir vehicle_vel_rot")
 
     do i = 1, 3
        write (fname, '("./vehicle_vel_rot/chassis_vel_", i3.3, ".txt")') i ! ここでファイル名を生成している
        open(100+i, file = trim(fname), status = "unknown")
    end do

    do i = 1, 50
        write (fname, '("./vehicle_vel_rot/steering_angle_", i3.3, ".txt")') i ! ここでファイル名を生成している
        open(200+i, file = trim(fname), status = "unknown")
    end do

    do i = 1, 50
        write (fname, '("./vehicle_vel_rot/wheel_rot_", i3.3, ".txt")') i ! ここでファイル名を生成している
        open(300+i, file = trim(fname), status = "unknown")        
    end do 


    return
end subroutine file_open


subroutine file_write(cube_time,  mesh_vel_acc, chassis_vel_cube, str_vel_cube, wheel_rot_cube )
    implicit none
    integer :: output_id, i
    real(8),intent(in) :: cube_time
    real(8),dimension(6),intent(in) :: mesh_vel_acc
    real(8),dimension(3,6),intent(in) :: chassis_vel_cube
    real(8),dimension(50,6),intent(in) :: str_vel_cube, wheel_rot_cube

    do i = 1, 3
        write(100+i,'(7f15.5)') cube_time, chassis_vel_cube(i,1), chassis_vel_cube(i,2), &
        chassis_vel_cube(i,3), chassis_vel_cube(i,4), chassis_vel_cube(i,5), &
        chassis_vel_cube(i,6)
    end do

    do i = 1, 50
        write(200+i,'(7f15.5)') cube_time, str_vel_cube(i,1), str_vel_cube(i,2), &
        str_vel_cube(i,3), str_vel_cube(i,4), str_vel_cube(i,5), &
        str_vel_cube(i,6)
    end do 


    do i = 1, 50
        write(300+i,'(7f15.5)') cube_time, wheel_rot_cube(i,1), wheel_rot_cube(i,2), &
        wheel_rot_cube(i,3), wheel_rot_cube(i,4), wheel_rot_cube(i,5), &
        wheel_rot_cube(i,6)
    end do

    return
end subroutine file_write

subroutine file_close()
    implicit none
    integer :: i

    close(10)

    do i = 1, 3
        close(100+i)
    end do

    do i = 1, 50
        close(200+i)
    end do

    do i = 1, 50
        close(300+i)
    end do

    return
end subroutine file_close

