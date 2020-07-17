import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import time

from wsm_simulations.msg import PendelumState
from wsm_simulations.msg import PendelumMotor

qtable_shape = (9, 15, 15, )
plot_update_interval = 0.5

qtable = np.empty(qtable_shape, dtype=np.float32)
qtable[:] = np.nan

qpolicy = np.empty(qtable_shape, dtype=np.float32)
qpolicy[:] = np.nan

state_commonality = np.zeros(qtable_shape, dtype=np.float32)
recent_states = np.zeros(qtable_shape, dtype=np.float32)

last_state = ()
last_plot_time = time.time()

count = 0

def update_plot(pendelum_motor):
    global last_state, last_plot_time, plot_update_interval, count, state_commonality, recent_states

    velocity = pendelum_motor.state[0]
    angular_velocity = pendelum_motor.state[1]
    angle = pendelum_motor.state[2]

    if last_state != ():
        qtable[last_state] = pendelum_motor.qvalue
    last_state = (velocity, angular_velocity, angle)

    last_qpolicy = qpolicy[last_state]
    if np.isnan(last_qpolicy):
        qpolicy[last_state] = pendelum_motor.motor_response
    else:
        qpolicy[last_state] = last_qpolicy*0.85 + pendelum_motor.motor_response*0.15

    state_commonality[last_state] += 1
    state_commonality *= 0.99999
    recent_states[last_state] = 1
    recent_states *= 0.8

    if plot_update_interval < time.time()-last_plot_time:
        if update_plot.first_time:
            update_plot.first_time = False
            update_plot.fig, (update_plot.ax_qtable, update_plot.ax_qpolicy, update_plot.ax_commonality) = plt.subplots(3, 1)
            update_plot.im_qtable = update_plot.ax_qtable.imshow(np.hstack(qtable),
                                                          origin='bottom',
                                                          #aspect='auto',
                                                          vmin=-4,
                                                          vmax=4.0,
                                                          cmap='bone')

            update_plot.im_qpolicy = update_plot.ax_qpolicy.imshow(np.hstack(qpolicy),
                                                                 origin='bottom',
                                                                 #aspect='auto',
                                                                 vmin=-0.8,
                                                                 vmax=0.8,
                                                                 cmap='RdBu')

            update_plot.im_common = update_plot.ax_commonality.imshow(np.hstack(state_commonality),
                                                                   origin='bottom',
                                                                   #aspect='auto',
                                                                   vmin=0.0,
                                                                   vmax=1.5,
                                                                   cmap='magma')

            update_plot.ax_qtable.set_title("Q_table")
            update_plot.ax_qpolicy.set_title("Q_policy")
            update_plot.ax_commonality.set_title("Commonality")
            update_plot.ax_commonality.set_xlabel("velocity[angle]")
            update_plot.ax_qpolicy.set_ylabel("angular_velocity")


        not_nans = np.where(np.isnan(qtable) == False)

        mqtable = np.ma.masked_where(np.isnan(qtable), qtable)
        mqtable = mqtable-np.mean(mqtable[not_nans])
        mqtable = mqtable/np.std(mqtable[not_nans])

        mcommonality = np.log(state_commonality+1)
        mcommonality = mcommonality/np.max(mcommonality)

        mqpolicy = np.ma.masked_where(np.isnan(qpolicy), qpolicy)\
                   *(mcommonality+0.05)
        mqpolicy = mqpolicy-np.mean(mqpolicy[not_nans])
        mqpolicy = mqpolicy/(np.max(mqpolicy[not_nans])-np.min(mqpolicy[not_nans]))

        update_plot.im_qtable.set_data(np.hstack(mqtable))
        update_plot.im_qpolicy.set_data(np.hstack(mqpolicy))
        update_plot.im_common.set_data(np.hstack(mcommonality+recent_states))

        if count%50 == 0:
            plt.savefig("./qtable_animation/"+str(count)+".jpg")
        plt.draw()
        plt.pause(0.0001)

        last_plot_time = time.time()
        count += 1

update_plot.first_time = True;

def listener():
    rospy.init_node('plot_pendelum_qtable', anonymous=True)
    rospy.Subscriber('pendelum_motor', PendelumMotor, update_plot)
    print("Listening for pendelum_motor topic");
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
