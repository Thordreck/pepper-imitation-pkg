#!/usr/bin/env python
import rospy
import smach
import smach_ros

import pepper_imitation.msg
import std_msgs.msg
import tf

@smach.cb_interface(outcomes = ['finished'])
def say_hi(user_data):
    tts_publisher = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)
    rospy.sleep(1.0);

    tts_publisher.publish(std_msgs.msg.String("\\style=joyful\\ ^start(animations/Stand/Gestures/Hey_1) Bonjour, je m'appelle Pepper. Bienvenue a IMT Atlantique. Veuillez tapez sur l'ecran pour entrer le prenon de l'enfant ^wait(animations/Stand/Gestures/Hey_1)"))
    rospy.sleep(5.0);

    return 'finished'

class WaitUserInput(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start', 'stop', 'waiting', 'preempted'], output_keys = ['player_name'])
        self.tts_publisher = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)

    def execute(self, user_data):
        try:
            msg = rospy.wait_for_message('/pepper_imitation/cmd_user', pepper_imitation.msg.UserCommand, 1)
            if msg.command == pepper_imitation.msg.UserCommand.START_GAME:
                return 'start'
            elif msg.command == pepper_imitation.msg.UserCommand.STOP_GAME:
                return 'stop'
            elif msg.command == pepper_imitation.msg.UserCommand.INPUT_USER_DATA:
                user_data.player_name = msg.args;
                self.tts_publisher.publish(std_msgs.msg.String("\\style=joyful\\ Veuillez choisir la comptine"))
                return 'waiting'
        except rospy.ROSException:
            return 'waiting'

@smach.cb_interface(outcomes = ['finished'], input_keys=['player_name'], output_keys=['player_name'])
def init_game(user_data):
    face_tracking_publisher = rospy.Publisher('pepper_imitation/cmd_set_face_tracking', std_msgs.msg.Bool, queue_size = 1)
    audio_player_publisher  = rospy.Publisher('pepper_imitation/cmd_audio_player', pepper_imitation.msg.AudioPlayerCommand, queue_size = 1)
    rospy.sleep(1.0);

    audio_player_publisher.publish(pepper_imitation.msg.AudioPlayerCommand(command=pepper_imitation.msg.AudioPlayerCommand.PLAY, file="comptine.wav"))
    face_tracking_publisher.publish(std_msgs.msg.Bool(True))
    rospy.sleep(5.0);

    return 'finished'

class CheckPoseState(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes = ['finished', 'no_skeleton', 'preempted', 'waiting'], output_keys = ['detection_succeeded'])

    def execute(self, user_data):
        try:
            msg = rospy.wait_for_message('/pepper_imitation/imitation_result', pepper_imitation.msg.ImitationResult, 1)
            user_data.detection_succeeded = (msg.result == pepper_imitation.msg.ImitationResult.SUCCESS);

            if msg.result == pepper_imitation.msg.ImitationResult.NO_SKELETON:
                return 'no_skeleton'
            else:
                return 'finished'
        except rospy.ROSException:
            return 'waiting'

class GameIteration(smach.State) :
    def __init__(self):
        self.audio_player_publisher = rospy.Publisher('pepper_imitation/cmd_audio_player', pepper_imitation.msg.AudioPlayerCommand, queue_size = 1)
        self.tts_publisher          = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)
        smach.State.__init__(self, outcomes = ['continue', 'game_over', 'preempted'], input_keys = ['previous_imitation_succeeded'], output_keys = ['synchro_time', 'next_pose', 'score', 'player_name'])
        self.game_sections = [{'start_time': 20,  'pose': pepper_imitation.msg.ImitationPose.HANDS_UP},
                              {'start_time': 43,  'pose': pepper_imitation.msg.ImitationPose.HANDS_ON_HEAD},
                              {'start_time': 65,  'pose': pepper_imitation.msg.ImitationPose.HANDS_ON_SHOULDERS},
                              {'start_time': 88,  'pose': pepper_imitation.msg.ImitationPose.HANDS_ON_FRONT},
                              {'start_time': 110, 'pose': pepper_imitation.msg.ImitationPose.CROSSED_ARMS},
                              {'start_time': 132, 'pose': pepper_imitation.msg.ImitationPose.HANDS_ON_SIDES},
                              {'start_time': 154, 'pose': pepper_imitation.msg.ImitationPose.HANDS_TOGETHER},
                              {'start_time': 175, 'pose': pepper_imitation.msg.ImitationPose.HAND_ON_MOUTH}]

        self.current_section = 0;
        self.num_iterations = 0;

    def execute(self, user_data):
        if (self.num_iterations == 0):
            user_data.score = 0;

        if (self.num_iterations > 0) and ('previous_imitation_succeeded' in user_data)  and (user_data.previous_imitation_succeeded == True):
            self.current_section = self.current_section + 1
            if self.current_section >= len(self.game_sections):
                self.current_section = 0
                self.num_iterations  = 0
                return 'game_over'

        self.num_iterations     = self.num_iterations + 1;
        user_data.synchro_time  = self.game_sections[self.current_section]['start_time']
        user_data.next_pose     = self.game_sections[self.current_section]['pose']

        self.tts_publisher.publish(std_msgs.msg.String("\\style=joyful\\ Fait comme moi!"))
        self.audio_player_publisher.publish(pepper_imitation.msg.AudioPlayerCommand(command=pepper_imitation.msg.AudioPlayerCommand.GOTO, time=self.game_sections[self.current_section]['start_time']))
        return 'continue'

def synchronize_song(user_data, message):
    return message.data < user_data.synchro_time; 

@smach.cb_interface(outcomes = ['finished'], input_keys = ['pose'])
def send_pose(user_data):
    pose_publisher = rospy.Publisher('pepper_imitation/cmd_set_pose', pepper_imitation.msg.ImitationPose, queue_size = 1)
    rospy.sleep(1.0);
    pose_publisher.publish(pepper_imitation.msg.ImitationPose(pose = user_data.pose, timeout = 15))
    return 'finished'

@smach.cb_interface(outcomes = ['finished'], input_keys = ['positive_feedback', 'player_name', 'score'])
def give_feedback(user_data):
    tts_publisher          = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)
    audio_player_publisher = rospy.Publisher('pepper_imitation/cmd_audio_player', pepper_imitation.msg.AudioPlayerCommand, queue_size = 1)
    rospy.sleep(1.0);

    if(user_data.positive_feedback):
        user_data.score += 20;
    else:
        user_data.score -= 5;

    audio_player_publisher.publish(pepper_imitation.msg.AudioPlayerCommand(command=pepper_imitation.msg.AudioPlayerCommand.PAUSE))
    rospy.sleep(1.0)
    tts_publisher.publish(std_msgs.msg.String("\\style=joyful\\ ^start(animations/Stand/Emotions/Positive/Happy_4) Bravo, bravo" + user_data.player_name + "! ^wait(animations/Stand/Emotions/Positive/Happy_4)" if user_data.positive_feedback else "\\style=didactic\\ ^start(animations/Stand/Gestures/Explain_1)"  + "Allez " + user_data.player_name + ", regarde moi. Encore une fois! ^wait(animations/Stand/Gestures/Explain_1)"))
    rospy.sleep(5.0)
    return 'finished'

@smach.cb_interface(outcomes = ['finished'], input_keys=['player_name', 'score'])
def end_session(user_data):
    face_tracking_publisher = rospy.Publisher('pepper_imitation/cmd_set_face_tracking', std_msgs.msg.Bool, queue_size = 1)
    tts_publisher = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)
    rospy.sleep(1.0);

    tts_publisher.publish(std_msgs.msg.String("\\style=joyful\\" + "Bravo " + user_data.player_name + ", c'est tres bien! Je me suis bien amuse avec toi. A bientot"))
    rospy.sleep(4.0);
    face_tracking_publisher.publish(std_msgs.msg.Bool(False))
    return 'finished'

@smach.cb_interface(outcomes = ['finished'], input_keys=['player_name'])
def game_stopped(user_data):
    tts_publisher = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)
    rospy.sleep(1.0);

    tts_publisher.publish(std_msgs.msg.String("\\style=joyful\\ Au revoir! A la prochaine fois"))
    rospy.sleep(4.0);
    return 'finished'

def check_user_exit(user_data, message):
    return message.command != pepper_imitation.msg.UserCommand.EXIT_GAME; 

def game_termination_cb(outcome_map):
    if outcome_map['USER_EXIT_GAME'] != 'invalid':
        return False
    return True

def game_outcome_cb(outcome_map):
    if outcome_map['USER_EXIT_GAME'] == 'invalid':
        return 'game_canceled'
    elif outcome_map['GAME'] == main_success:
        return 'game_success'
    return 'game_error'

class WaitSkeletonState(smach.State) :
    def __init__(self):
        smach.State.__init__(self, outcomes = ['skeleton_found', 'skeleton_not_found', 'waiting', 'preempted'])
        self.tf_listener = tf.TransformListener()
        self.start_time = None

    def execute(self, user_data):
        if self.start_time == None:
            self.start_time = rospy.Time.now()

        frames = self.tf_listener.getFrameStrings()
        for frame in frames:
            rospy.logerr("TF frame name: " + frame)
            if frame.startswith('torso_'):
                self.start_time = None
                return 'skeleton_found'

        if rospy.Time.now() - self.start_time <= rospy.Duration(10):
            return 'waiting'

        self.start_time = None
        return 'skeleton_not_found'

@smach.cb_interface(outcomes = ['finished'], input_keys=['player_name'])
def skeleton_found(user_data):
    tts_publisher = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)
    rospy.sleep(1.0);
    tts_publisher.publish(std_msgs.msg.String("\\style=joyful\\ Rebonjour. Vas-y on recommence"))
    rospy.sleep(4.0);
    return 'finished'

@smach.cb_interface(outcomes = ['finished'])
def skeleton_not_found(user_data):
    tts_publisher = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)
    rospy.sleep(1.0);
    tts_publisher.publish(std_msgs.msg.String("\\style=didactic\\ ^start(animations/Stand/Gesture/Desperate_1) Je n'ai pas pu vous detecter. Veuillez recommencer ulterieurement ^wait(animations/Stand/Gesture/Desperate_1)"))
    rospy.sleep(4.0);
    return 'finished'

@smach.cb_interface(outcomes = ['finished'], input_keys=['player_name'])
def wait_skeleton_init(user_data):
    audio_player_publisher = rospy.Publisher('pepper_imitation/cmd_audio_player', pepper_imitation.msg.AudioPlayerCommand, queue_size = 1)
    tts_publisher = rospy.Publisher('pepper_imitation/cmd_say', std_msgs.msg.String, queue_size = 1)
    rospy.sleep(1.0);
    audio_player_publisher.publish(pepper_imitation.msg.AudioPlayerCommand(command=pepper_imitation.msg.AudioPlayerCommand.STOP))
    tts_publisher.publish(std_msgs.msg.String("\\style=didactic\\ Je ne peux pas vous detecter!"))
    rospy.sleep(4.0);
    return 'finished'

def main():
    rospy.init_node("pepper_imitation_game_node")

    top_sm = smach.Concurrence(outcomes = ['game_success', 'game_error', 'game_canceled'],
                               default_outcome = 'game_error', child_termination_cb = game_termination_cb, outcome_cb = game_outcome_cb)

    with top_sm:
        main_sm = smach.StateMachine(outcomes = ['main_success', 'main_failed', 'main_preempted'])

        with main_sm:
            smach.StateMachine.add('SAY_HI', smach.CBState(say_hi), transitions = {'finished' : 'WAIT_USER_INPUT'})
            smach.StateMachine.add('WAIT_USER_INPUT', WaitUserInput(), transitions={ 'start':'INIT_GAME', 'stop':'main_success', 'preempted': 'GAME_STOPPED', 'waiting':'WAIT_USER_INPUT'})
            smach.StateMachine.add('INIT_GAME', smach.CBState(init_game), transitions = {'finished' : 'GAME_ITERATION'})
            smach.StateMachine.add('GAME_ITERATION', GameIteration(), transitions={ 'continue':'SYNC_MUSIC', 'game_over':'END_SESSION', 'preempted': 'GAME_STOPPED'},
                                    remapping={'previous_imitation_succeeded':'game_state_result',
                                               'synchro_time':'game_state_synchro_time',
                                               'next_pose':'game_state_pose'})

            smach.StateMachine.add('SYNC_MUSIC', smach_ros.MonitorState("pepper_imitation/audio_player_progress", \
                    std_msgs.msg.Float32, synchronize_song, input_keys = ['synchro_time']), transitions = {'valid':'SYNC_MUSIC', 'invalid':'SEND_POSE', 'preempted': 'GAME_STOPPED'}, 
                    remapping = {'synchro_time':'game_state_synchro_time'})

            smach.StateMachine.add('SEND_POSE', smach.CBState(send_pose), transitions = {'finished' : 'CHECK_POSE'}, remapping = {'pose':'game_state_pose'})
            smach.StateMachine.add('CHECK_POSE', CheckPoseState(), transitions={'waiting':'CHECK_POSE', 'finished':'GIVE_FEEDBACK', 'no_skeleton':'GET_SKELETON', 'preempted': 'GAME_STOPPED'}, remapping = {'detection_succeeded':'game_state_result'}) 
            smach.StateMachine.add('GIVE_FEEDBACK', smach.CBState(give_feedback), transitions = {'finished' : 'GAME_ITERATION'}, remapping = {'positive_feedback':'game_state_result'})
            smach.StateMachine.add('END_SESSION', smach.CBState(end_session), transitions = {'finished' : 'WAIT_USER_INPUT'})
            smach.StateMachine.add('GAME_STOPPED', smach.CBState(game_stopped), transitions = {'finished' : 'main_preempted'})

            skeleton_sm = smach.StateMachine(outcomes = ['skeleton_found', 'timeout', 'preempted'])

            with skeleton_sm:
                smach.StateMachine.add('WAIT_SKELETON_INIT', smach.CBState(wait_skeleton_init), transitions = {'finished' : 'WAIT_SKELETON'})
                smach.StateMachine.add('WAIT_SKELETON', WaitSkeletonState(), transitions={ 'skeleton_found':'SKELETON_FOUND', 'skeleton_not_found':'SKELETON_NOT_FOUND', 'waiting':'WAIT_SKELETON', 'preempted': 'preempted' })
                smach.StateMachine.add('SKELETON_FOUND', smach.CBState(skeleton_found), transitions = {'finished' : 'skeleton_found'})
                smach.StateMachine.add('SKELETON_NOT_FOUND', smach.CBState(skeleton_not_found), transitions = {'finished' : 'timeout'})

            smach.StateMachine.add("GET_SKELETON", skeleton_sm, transitions = { 'skeleton_found':'INIT_GAME', 'timeout':'main_failed', 'preempted':'GAME_STOPPED' })

        smach.Concurrence.add('GAME', main_sm)
        smach.Concurrence.add('USER_EXIT_GAME', smach_ros.MonitorState("/pepper_imitation/cmd_user", pepper_imitation.msg.UserCommand, check_user_exit))

    introspection_server = smach_ros.IntrospectionServer('pepper_imitation_game_state', top_sm, '/PEPPER_IMITATION_ROOT')
    introspection_server.start()

    outcome = top_sm.execute();
    rospy.spin()
    introspection_server.stop();

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
