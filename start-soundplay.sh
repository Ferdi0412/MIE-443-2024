##################
### GET PARAMS ###
while [ $# -gt 0 ]; do
    case "$1" in
        -h | --help)
            echo "Usage: $0 [NO OPTIONS]"
            echo "Starts the sound_play node"
            exit 0
            ;;
    esac
done

command="rosrun sound_play soundplay_node.py"
echo ""
echo "$command"
echo ""
eval "$command"