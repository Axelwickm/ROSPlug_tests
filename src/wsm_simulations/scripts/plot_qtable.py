import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import time

from wsm_simulations.msg import QTable

count = 0
def update(qtable):
    global count

    data = qtable.data
    if len(data) == 0:
        return
    # Get settings
    explored_states = data[0] # State + action
    action_count = data[1]
    state_count = data[2]
    states_tuple = tuple([data[i] for i in range(4, 4+state_count)])

    # Extract values
    _data = np.array(data[4+state_count:], dtype=np.int32).reshape((explored_states, 4+state_count))
    _qvalues = _data[:, 0:2].copy().view('d').flatten()
    _explored_count = _data[:, 2].view('<u4')
    _actions = _data[:, 3].view('<u4')
    _states = _data[:, 4:].view('<u4')
    #_full_states = np.concatenate((_states, _actions.reshape(-1, 1)), axis=1)

    # Build data to show
    qtable = np.empty(states_tuple, dtype=np.float32)
    qtable[:] = np.nan
    qtable[tuple(_states.T)] = _qvalues

    qpolicy = np.empty(states_tuple, dtype=np.float32)
    qpolicy[tuple(_states.T)] = _actions

    commonality = np.zeros(states_tuple, dtype=np.float32)
    commonality[tuple(_states.T)] = _explored_count


    # Init plots
    if update.first_time:
        update.first_time = False
        update.fig, (update.ax_qtable, update.ax_qpolicy, update.ax_commonality) = plt.subplots(3, 1)
        update.im_qtable = update.ax_qtable.imshow(np.hstack(qtable),
                                                             origin='bottom',
                                                             #aspect='auto',
                                                             vmin=-4,
                                                             vmax=4.0,
                                                             cmap='bone')

        update.im_qpolicy = update.ax_qpolicy.imshow(np.hstack(qpolicy),
                                                               origin='bottom',
                                                               #aspect='auto',
                                                               vmin=-0.8,
                                                               vmax=0.8,
                                                               cmap='RdBu')

        update.im_common = update.ax_commonality.imshow(np.hstack(commonality),
                                                                  origin='bottom',
                                                                  #aspect='auto',
                                                                  vmin=0.0,
                                                                  vmax=1.5,
                                                                  cmap='magma')

        update.ax_qtable.set_title("Q_table")
        update.ax_qpolicy.set_title("Q_policy")
        update.ax_commonality.set_title("Commonality")
        update.ax_commonality.set_xlabel("velocity[angle]")
        update.ax_qpolicy.set_ylabel("angular_velocity")



    not_nans = np.where(np.isnan(qtable) == False)

    mqtable = np.ma.masked_where(np.isnan(qtable), qtable)
    mqtable = mqtable-np.mean(mqtable[not_nans])
    mqtable = mqtable/np.std(mqtable[not_nans])
    update.im_qtable.set_data(np.hstack(mqtable))

    mqpolicy = np.ma.masked_where(np.isnan(qtable), qpolicy)
    mqpolicy = mqpolicy-np.mean(mqpolicy[not_nans])
    mqpolicy = mqpolicy/(np.max(mqpolicy[not_nans])-np.min(mqpolicy[not_nans]))
    update.im_qpolicy.set_data(np.hstack(mqpolicy))

    mcommonality = np.log(commonality+1)
    mcommonality = mcommonality/np.max(mcommonality)
    update.im_common.set_data(np.hstack(mcommonality))

    # Show data and save image
    #plt.ion()
    plt.draw()
    plt.pause(0.0001)
    plt.savefig("./qtable_animation/"+str(count)+".jpg")

    count += 1

update.first_time = True

def listener():
    rospy.init_node('plot_pendelum_qtable', anonymous=True)
    rospy.Subscriber('qtable', QTable, update)
    print("Listening for qtable topic");
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass