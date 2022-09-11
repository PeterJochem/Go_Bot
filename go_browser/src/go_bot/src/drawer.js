import * as React from 'react';
import { styled, useTheme } from '@mui/material/styles';
import Box from '@mui/material/Box';
import Drawer from '@mui/material/Drawer';
import MuiAppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import CssBaseline from '@mui/material/CssBaseline';
import List from '@mui/material/List';
import Typography from '@mui/material/Typography';
import Divider from '@mui/material/Divider';
import IconButton from '@mui/material/IconButton';
import MenuIcon from '@material-ui/icons/Menu';
import ChevronLeftIcon from '@material-ui/icons/ChevronLeft';
import ChevronRightIcon from '@material-ui/icons/ChevronRight';
import ListItem from '@mui/material/ListItem';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';
import InfoIcon from '@material-ui/icons/Info';
import PlayArrow from '@material-ui/icons/PlayArrow';
import Stack from "@mui/material/Stack";
import Snackbar from "@mui/material/Snackbar";
import MuiAlert from "@mui/material/Alert";

const drawerWidth = 240;

let snackBarMessage = "";
let snackBarSeverity = "success";

const Alert = React.forwardRef(function Alert(props, ref) {
  return <MuiAlert elevation={6} ref={ref} variant="filled" {...props} />;
});

const Main = styled('main', { shouldForwardProp: (prop) => prop !== 'open' })(
  ({ theme, open }) => ({
    flexGrow: 1,
    padding: theme.spacing(3),
    transition: theme.transitions.create('margin', {
      easing: theme.transitions.easing.sharp,
      duration: theme.transitions.duration.leavingScreen,
    }),
    marginRight: -drawerWidth,
    ...(open && {
      transition: theme.transitions.create('margin', {
        easing: theme.transitions.easing.easeOut,
        duration: theme.transitions.duration.enteringScreen,
      }),
      marginRight: 0,
    }),
  }),
);

const AppBar = styled(MuiAppBar, {
  shouldForwardProp: (prop) => prop !== 'open',
})(({ theme, open }) => ({
  transition: theme.transitions.create(['margin', 'width'], {
    easing: theme.transitions.easing.sharp,
    duration: theme.transitions.duration.leavingScreen,
  }),
  ...(open && {
    width: `calc(100% - ${drawerWidth}px)`,
    transition: theme.transitions.create(['margin', 'width'], {
      easing: theme.transitions.easing.easeOut,
      duration: theme.transitions.duration.enteringScreen,
    }),
    marginRight: drawerWidth,
  }),
}));

const DrawerHeader = styled('div')(({ theme }) => ({
  display: 'flex',
  alignItems: 'center',
  padding: theme.spacing(0, 1),
  // necessary for content to be below app bar
  ...theme.mixins.toolbar,
  justifyContent: 'flex-start',
}));

export default function PersistentDrawerRight() {
  const theme = useTheme();
  const [open, setOpen] = React.useState(false);
  const [snackOpen, setSnackOpen] = React.useState(false);
  let project_details_link = "https://github.com/PeterJochem/Go_Bot";
 
  const handleDrawerOpen = () => {
    setOpen(true);
  };

  const handleDrawerClose = () => {
    setOpen(false);
  };
   
  const handleSnackClose = (event, reason) => {
    if (reason === "clickaway") {
      return;
    }

    setSnackOpen(false);
  };
  
  async function request_new_game() {
	
	let num_rows = 4;
	let num_cols = 4;
	let url = `http://localhost:8000/start_game?num_rows=${num_rows}&num_columns=${num_cols}&whitePlayerType=random&blackPlayerType=OnlineAgent`;
	let response = await fetch(url, {method: "POST"});
	let json = await response.json();
	let success = json.hasOwnProperty('success') && json['success'];

 	if (success) {
		snackBarSeverity="success"
		snackBarMessage="Starting New Game";	
	}
	else {
		snackBarSeverity = "error";
		snackBarMessage = "Unable to start a new game";
	}

	setTimeout(()=> {setSnackOpen(true);}, 1000);
  }
 

  return (
    <Box sx={{ display: 'flex'}}>
      <CssBaseline />
      <AppBar position="fixed" open={open}>
        <Toolbar sx={{ backgroundColor:"MediumBlue" }}>
          <Typography variant="h2" noWrap sx={{ flexGrow: 1 }} component="div">
            Go Bot
          </Typography>
          <IconButton
            color="inherit"
            aria-label="open drawer"
            edge="end"
            onClick={handleDrawerOpen}
            sx={{ ...(open && { display: 'none' }) }}
	  >
            <MenuIcon />
          </IconButton>
        </Toolbar>
      </AppBar>

      <Main open={open}>
        <DrawerHeader />
      </Main>
      <Drawer
        sx={{
          width: drawerWidth,
          flexShrink: 0,
          '& .MuiDrawer-paper': {
            width: drawerWidth,
          },
        }}
        variant="persistent"
        anchor="right"
        open={open}
      >
        <DrawerHeader>
          <IconButton onClick={handleDrawerClose}>
            {theme.direction === 'rtl' ? <ChevronLeftIcon /> : <ChevronRightIcon />}
          </IconButton>
        </DrawerHeader>
        <Divider />
        <List>
	<ListItem button key={'Request New Game'} onClick={request_new_game}>
              <ListItemIcon>
	      <PlayArrow />
	      </ListItemIcon>
              <ListItemText primary={'Request New Game'} />
            </ListItem>
	
	  <a href={project_details_link}>
	  <ListItem button key={"Project Details"}>
	      <ListItemIcon>
              <InfoIcon />
              </ListItemIcon>
              <ListItemText primary={"Project Details"} />
            </ListItem>
	  </a>

        </List>
        <Divider />
      </Drawer>
  
     <div>
        <Stack spacing={2} sx={{ width: "100%" }}>
          <Snackbar
            open={snackOpen}
            autoHideDuration={5000}
            onClose={handleSnackClose}
          >
            <Alert
              onClose={handleSnackClose}
              severity={snackBarSeverity}
              sx={{ width: "100%" }}
            >
              {snackBarMessage}
            </Alert>
          </Snackbar>
        </Stack>
      </div>

    </Box>
  );
}
